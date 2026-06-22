#pragma once

#if AP_PERIPH_SOLAR_BMS

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

/*
  Analog Devices LT8491 MPPT solar battery charger controller.

  The LT8491 exposes an I2C slave interface (7-bit address 0x29 on this board,
  selected by the CA-pin resistor network — datasheet Table 24) for charger
  control and telemetry. All word-sized registers are LITTLE ENDIAN: the LSB is
  at the lower I2C address (datasheet "Data: Bytes, Words and Long Words"), so a
  word read returns DATA0 = LSB then DATA1 = MSB.

  Power control: the LT8491 only runs while its SHDN pin is held high. On this
  board SHDN is driven by GPIO_SHDN. The deadman button (GPIO_DEADMAN_BUTTON,
  low = pressed) is momentary: the charger is powered while the button is held
  and shut down when released. GPIO_TEST_LED mirrors the commanded-on state.

  Once SHDN goes high the chip runs its startup/CRC sequence, copies the EEPROM
  boot region into the configuration registers, and (because BOOT_INIT_CHRG_EN
  defaults to 0) leaves the charging logic off. While the logic is off we apply
  the page-70 Lithium-Ion recommended settings (Table 26 — this board carries an
  8S LiPo) and then set CTRL_CHRG_EN to start charging.

  Telemetry (volts/amps/power/efficiency) is only valid once the board's PCB
  resistor values have been programmed into the CFG_RSENSE / CFG_RFB / CFG_RDAC
  registers (normally stored in EEPROM). When those are unset the chip returns
  0xFFFF for the affected telemetry, which this driver detects and flags.
*/
class SolarBMS {
public:
    friend class AP_Periph_FW;

    SolarBMS(uint8_t bus, uint8_t addr) : _bus(bus), _addr(addr) {}
    ~SolarBMS() { delete _dev; }

    void update(void);

private:
    // ===== LT8491 I2C register map (datasheet Rev.0, pages 27-28) =============
    // Telemetry — read-only, WORD, little-endian
    static constexpr uint8_t REG_TELE_TBAT = 0x00; // C*10,  signed   (sentinels below)
    static constexpr uint8_t REG_TELE_POUT = 0x02; // W*100, unsigned
    static constexpr uint8_t REG_TELE_PIN  = 0x04; // W*100, unsigned
    static constexpr uint8_t REG_TELE_EFF  = 0x06; // %*100, unsigned
    static constexpr uint8_t REG_TELE_IOUT = 0x08; // mA,    unsigned
    static constexpr uint8_t REG_TELE_IIN  = 0x0A; // mA,    unsigned
    static constexpr uint8_t REG_TELE_VBAT = 0x0C; // V*100, unsigned
    static constexpr uint8_t REG_TELE_VIN  = 0x0E; // V*100, unsigned (FBIR; 0 when solar)
    static constexpr uint8_t REG_TELE_VINR = 0x10; // V*100, unsigned (VINR; 0 when DC supply)
    // Status — read-only, BYTE
    static constexpr uint8_t REG_STAT_CHARGER     = 0x12;
    static constexpr uint8_t REG_STAT_SYSTEM      = 0x13;
    static constexpr uint8_t REG_STAT_SUPPLY      = 0x14;
    static constexpr uint8_t REG_STAT_CHRG_FAULTS = 0x19;
    // Control — read-write, BYTE
    static constexpr uint8_t REG_CTRL_CHRG_EN = 0x23;
    // Configuration — read-write, only writable while CHRG_LOGIC_ON=0
    static constexpr uint8_t REG_CFG_TBAT_MIN  = 0x40; // signed C
    static constexpr uint8_t REG_CFG_TBAT_MAX  = 0x41; // signed C
    static constexpr uint8_t REG_CFG_TMR_S0    = 0x42;
    static constexpr uint8_t REG_CFG_TMR_S1    = 0x43;
    static constexpr uint8_t REG_CFG_TMR_S2    = 0x44;
    static constexpr uint8_t REG_CFG_TMR_S3    = 0x45;
    static constexpr uint8_t REG_CFG_CHRG_MISC = 0x4D;

    // STAT_CHARGER (0x12) bit fields
    static constexpr uint8_t CHARGER_FAULT        = 1U << 7;
    static constexpr uint8_t CHARGER_TELEM_ACTIVE = 1U << 6;
    static constexpr uint8_t CHARGER_STAGE_MASK   = 0x7U << 3;
    static constexpr uint8_t CHARGER_STAGE_SHIFT  = 3;
    static constexpr uint8_t CHARGER_CHARGING     = 1U << 2;
    static constexpr uint8_t CHARGER_GT_C10       = 1U << 1;
    static constexpr uint8_t CHARGER_LOGIC_ON     = 1U << 0;
    static constexpr uint8_t CHRG_STAGE_DONE      = 0x4;  // 100b

    // STAT_SYSTEM (0x13) bit fields
    static constexpr uint8_t SYSTEM_BOOT_SUCCESS = 1U << 5;
    static constexpr uint8_t SYSTEM_BUSY_MASK    = 0x3U;

    // STAT_SUPPLY (0x14) bit fields
    static constexpr uint8_t SUPPLY_VIN_UVLO    = 1U << 4;
    static constexpr uint8_t SUPPLY_PS_OR_SOLAR = 1U << 3;  // 1 = DC supply, 0 = solar
    static constexpr uint8_t SUPPLY_SOLAR_STATE_MASK = 0x7U;

    // CTRL_CHRG_EN (0x23)
    static constexpr uint8_t CHRG_EN = 1U << 0;

    // 16-bit telemetry sentinels
    static constexpr uint16_t TELE_NOT_CONFIGURED = 0xFFFF; // a CFG_R* register is 0x0000
    static constexpr uint16_t TBAT_NOT_MEASURED   = 0x7FFF;
    static constexpr uint16_t TBAT_DISCONNECTED   = 0x7777;

    static constexpr uint32_t POLL_MS  = 100;   // telemetry/status poll period
    static constexpr uint32_t PRINT_MS = 500;   // USB console print period
    static constexpr uint32_t RETRY_MS = 1000;  // re-enable attempt period when stalled

    bool init(void);

    // little-endian word read / byte read / byte write, each taking the bus semaphore
    bool read_word(uint8_t reg, uint16_t &val);
    bool read_byte(uint8_t reg, uint8_t &val);
    bool write_byte(uint8_t reg, uint8_t val);

    void apply_config(void);            // page-70 Lithium-Ion recommended settings
    void service(void);                 // I2C state machine while commanded on
    void print_status(void);            // dump telemetry + status over USB

    AP_HAL::I2CDevice *_dev;
    bool _initialised;

    const uint8_t _bus;
    const uint8_t _addr;

    // command / power state
    bool _commanded_on;
    bool _prev_deadman;
    bool _booted;        // chip startup complete and I2C responding
    bool _configured;    // page-70 config applied this power cycle
    bool _comms_ok;      // last poll's I2C reads succeeded

    uint32_t _last_poll_ms;
    uint32_t _last_print_ms;
    uint32_t _last_retry_ms;

    // latest raw telemetry / status registers
    uint16_t _raw_tbat, _raw_pout, _raw_pin, _raw_eff;
    uint16_t _raw_iout, _raw_iin, _raw_vbat, _raw_vin, _raw_vinr;
    uint8_t  _stat_charger, _stat_system, _stat_supply, _stat_faults;
};

#endif // AP_PERIPH_SOLAR_BMS
