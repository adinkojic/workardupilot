/*
    Stratospheres, Inc solar BMS board, based on LT8491 configured for an 8S LiPo

    Analog Devices LT8491 MPPT solar charger controller, driven over I2C.

    Power: the deadman button (GPIO_DEADMAN_BUTTON, low = pressed) is momentary —
    while held it drives SHDN high to power the LT8491, and releasing it pulls
    SHDN low to shut the charger down. GPIO_TEST_LED mirrors the commanded-on
    state. After the chip powers up we apply the datasheet page-70 Lithium-Ion
    recommended settings (Table 26) and enable charging via CTRL_CHRG_EN.

    Telemetry and status are polled and printed over the USB console, in the same
    style as bldc_motor.cpp. If the charger is commanded on but reports that it is
    not charging (and has not reached the done-charging stage), we re-assert the
    charge-enable bit once per second and print the fault register.

    All word registers are little-endian (LSB at the lower I2C address) per the
    datasheet "Data: Bytes, Words and Long Words" section.
*/

#include "AP_Periph.h"

#if AP_PERIPH_SOLAR_BMS

#include "solar_bms.h"
#include "hal.h"

extern const AP_HAL::HAL &hal;

// ---------------------------------------------------------------------------
// I2C helpers — each takes the bus semaphore for the duration of the transfer
// ---------------------------------------------------------------------------

bool SolarBMS::read_word(uint8_t reg, uint16_t &val)
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    uint8_t b[2];
    if (!_dev->read_registers(reg, b, sizeof(b))) {
        return false;
    }
    // little-endian: DATA0 (LSB) at reg, DATA1 (MSB) at reg+1
    val = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
    return true;
}

bool SolarBMS::read_byte(uint8_t reg, uint8_t &val)
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    return _dev->read_registers(reg, &val, 1);
}

bool SolarBMS::write_byte(uint8_t reg, uint8_t val)
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    return _dev->write_register(reg, val);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

bool SolarBMS::init(void)
{
    _dev = hal.i2c_mgr->get_device_ptr(_bus, _addr);
    if (_dev == nullptr) {
        return false;
    }
    _dev->set_retries(2);

    // Start powered down; the deadman button brings the charger up.
    hal.gpio->write(GPIO_SHDN, 0);
    hal.gpio->write(GPIO_TEST_LED, 0);
    return true;
}

// Page-70 "Lithium-Ion Battery Register Settings" (datasheet Table 26), applied
// for the 8S LiPo. These configuration registers are only writable while the
// charging logic is off (CHRG_LOGIC_ON=0), which is the case immediately after
// power-up since BOOT_INIT_CHRG_EN defaults to 0.
void SolarBMS::apply_config(void)
{
    write_byte(REG_CFG_TBAT_MIN, 0x00);   // low-temp fault limit = 0 C
    write_byte(REG_CFG_TBAT_MAX, 0x32);   // high-temp fault limit = 50 C
    write_byte(REG_CFG_TMR_S0, 0x00);     // all stage timers disabled
    write_byte(REG_CFG_TMR_S1, 0x00);
    write_byte(REG_CFG_TMR_S2, 0x00);
    write_byte(REG_CFG_TMR_S3, 0x00);

    // CFG_CHRG_MISC[2:0] = 000b: Stage-3 charging disabled (DC supply and solar)
    // and temperature compensation off. Preserve the upper bits (LPMODE_EN /
    // USE_VS3_IN_STAGE2) via read-modify-write.
    uint8_t misc;
    if (read_byte(REG_CFG_CHRG_MISC, misc)) {
        misc &= ~0x07;
        write_byte(REG_CFG_CHRG_MISC, misc);
    }
}

// ---------------------------------------------------------------------------
// I2C state machine — runs while the charger is commanded on
// ---------------------------------------------------------------------------

void SolarBMS::service(void)
{
    const uint32_t now = AP_HAL::millis();

    // Wait for the chip to finish its power-up / CRC startup before touching the
    // configuration. SHDN may have only just gone high, so a failed read here
    // simply means "not ready yet".
    if (!_booted) {
        uint8_t sys;
        if (!read_byte(REG_STAT_SYSTEM, sys)) {
            _comms_ok = false;
            return;
        }
        _comms_ok = true;
        _stat_system = sys;
        if ((sys & SYSTEM_BUSY_MASK) != 0 || !(sys & SYSTEM_BOOT_SUCCESS)) {
            return;   // still busy with startup
        }
        _booted = true;
    }

    // Once booted, apply the page-70 config (logic must be off) and start charging.
    if (!_configured) {
        uint8_t chg;
        if (read_byte(REG_STAT_CHARGER, chg) && !(chg & CHARGER_LOGIC_ON)) {
            apply_config();
        }
        write_byte(REG_CTRL_CHRG_EN, CHRG_EN);
        _configured    = true;
        _last_retry_ms = now;
    }

    // Read all telemetry + status registers.
    bool ok = true;
    ok &= read_word(REG_TELE_TBAT, _raw_tbat);
    ok &= read_word(REG_TELE_POUT, _raw_pout);
    ok &= read_word(REG_TELE_PIN,  _raw_pin);
    ok &= read_word(REG_TELE_EFF,  _raw_eff);
    ok &= read_word(REG_TELE_IOUT, _raw_iout);
    ok &= read_word(REG_TELE_IIN,  _raw_iin);
    ok &= read_word(REG_TELE_VBAT, _raw_vbat);
    ok &= read_word(REG_TELE_VIN,  _raw_vin);
    ok &= read_word(REG_TELE_VINR, _raw_vinr);
    ok &= read_byte(REG_STAT_CHARGER,     _stat_charger);
    ok &= read_byte(REG_STAT_SYSTEM,      _stat_system);
    ok &= read_byte(REG_STAT_SUPPLY,      _stat_supply);
    ok &= read_byte(REG_STAT_CHRG_FAULTS, _stat_faults);
    _comms_ok = ok;

    if (!ok) {
        return;
    }

    // Retry: if commanded on but not charging and not in the done-charging stage,
    // the charger has stalled (a fault that isn't auto-restarting, or it never
    // started). Re-assert the charge-enable bit once per second and report.
    const uint8_t stage   = (_stat_charger & CHARGER_STAGE_MASK) >> CHARGER_STAGE_SHIFT;
    const bool    charging = (_stat_charger & CHARGER_CHARGING) != 0;
    const bool    done     = (stage == CHRG_STAGE_DONE);
    if (!charging && !done && (now - _last_retry_ms >= RETRY_MS)) {
        _last_retry_ms = now;
        write_byte(REG_CTRL_CHRG_EN, CHRG_EN);
        hal.console->printf("LT8491: not charging (faults 0x%02X) - re-enabling charge\n",
                            (unsigned)_stat_faults);
    }
}

// ---------------------------------------------------------------------------
// Console output (USB CDC), mirroring the bldc_motor.cpp style
// ---------------------------------------------------------------------------

static const char *stage_name(uint8_t stage)
{
    switch (stage) {
    case 0: return "S0-trickle";
    case 1: return "S1-CC";
    case 2: return "S2-CV";
    case 3: return "S3-float";
    case 4: return "Done";
    default: return "?";
    }
}

void SolarBMS::print_status(void)
{
    if (!_comms_ok) {
        hal.console->printf("LT8491: SHDN on but no I2C response (bus %u addr 0x%02X)\n",
                            (unsigned)_bus, (unsigned)_addr);
        return;
    }
    if (!_booted) {
        hal.console->printf("LT8491: powering up (STAT_SYSTEM 0x%02X)...\n",
                            (unsigned)_stat_system);
        return;
    }

    const uint8_t stage    = (_stat_charger & CHARGER_STAGE_MASK) >> CHARGER_STAGE_SHIFT;
    const bool    charging = (_stat_charger & CHARGER_CHARGING) != 0;
    const bool    logic_on = (_stat_charger & CHARGER_LOGIC_ON) != 0;
    const bool    telem_on = (_stat_charger & CHARGER_TELEM_ACTIVE) != 0;
    const bool    fault    = (_stat_charger & CHARGER_FAULT) != 0;
    const bool    dc_supply = (_stat_supply & SUPPLY_PS_OR_SOLAR) != 0;

    hal.console->printf("LT8491: %s  stage=%s  charging=%u logic=%u telem=%u gt_c10=%u  supply=%s%s\n",
                        fault ? "FAULT" : "ok",
                        stage_name(stage),
                        (unsigned)charging,
                        (unsigned)logic_on,
                        (unsigned)telem_on,
                        (unsigned)((_stat_charger & CHARGER_GT_C10) != 0),
                        dc_supply ? "DC" : "solar",
                        (_stat_supply & SUPPLY_VIN_UVLO) ? " VIN_UVLO" : "");

    // Battery temperature — signed C*10 with disconnect / not-measured sentinels.
    if (_raw_tbat == TBAT_DISCONNECTED) {
        hal.console->printf("  TBAT: battery disconnected\n");
    } else if (_raw_tbat == TBAT_NOT_MEASURED) {
        hal.console->printf("  TBAT: not measured yet\n");
    } else {
        hal.console->printf("  TBAT: %.1f C\n", (double)((int16_t)_raw_tbat) / 10.0);
    }

    // Volts / amps / power / efficiency. A 0xFFFF reading means the relevant
    // CFG_R* PCB-resistor registers have not been programmed.
    const bool tele_cfg = (_raw_vbat != TELE_NOT_CONFIGURED);
    if (!tele_cfg) {
        hal.console->printf("  telemetry config registers unset "
                            "(program CFG_RSENSE*/CFG_RFB*/CFG_RDAC* for V/I/P/EFF)\n");
    }
    hal.console->printf("  VBAT: %.2f V  VIN: %.2f V  VINR: %.2f V\n",
                        (double)_raw_vbat / 100.0,
                        (double)_raw_vin  / 100.0,
                        (double)_raw_vinr / 100.0);
    hal.console->printf("  IIN: %.3f A  IOUT: %.3f A  PIN: %.2f W  POUT: %.2f W  EFF: %.2f %%\n",
                        (double)_raw_iin  / 1000.0,
                        (double)_raw_iout / 1000.0,
                        (double)_raw_pin  / 100.0,
                        (double)_raw_pout / 100.0,
                        (double)_raw_eff  / 100.0);

    if (_stat_faults != 0) {
        hal.console->printf("  FAULTS 0x%02X:%s%s%s%s%s%s%s%s\n",
                            (unsigned)_stat_faults,
                            (_stat_faults & (1U<<0)) ? " LOW_VBAT"   : "",
                            (_stat_faults & (1U<<1)) ? " LOW_TBAT"   : "",
                            (_stat_faults & (1U<<2)) ? " HIGH_TBAT"  : "",
                            (_stat_faults & (1U<<3)) ? " BAT_DISCON" : "",
                            (_stat_faults & (1U<<4)) ? " TS0_EXP"    : "",
                            (_stat_faults & (1U<<5)) ? " TS1_EXP"    : "",
                            (_stat_faults & (1U<<6)) ? " TS2_EXP"    : "",
                            (_stat_faults & (1U<<7)) ? " TS3_EXP"    : "");
    }
}

// ---------------------------------------------------------------------------
// Main update — called from the AP_Periph loop
// ---------------------------------------------------------------------------

void SolarBMS::update(void)
{
    if (!_initialised) {
        _initialised = true;
        if (!init()) {
            return;
        }
    }
    if (_dev == nullptr) {
        return;
    }

    // Deadman button: low = pressed. Momentary — charger powered only while held.
    const bool deadman = (hal.gpio->read(GPIO_DEADMAN_BUTTON) == 0);

    if (deadman && !_prev_deadman) {
        // rising edge: power the LT8491 up and (re)start its bring-up sequence
        _commanded_on = true;
        _booted       = false;
        _configured   = false;
        _comms_ok     = false;
        hal.gpio->write(GPIO_SHDN, 1);
    } else if (!deadman && _prev_deadman) {
        // falling edge: shut the charger down
        _commanded_on = false;
        hal.gpio->write(GPIO_SHDN, 0);
    }
    _prev_deadman = deadman;

    // TEST_LED on whenever the charger is commanded on.
    hal.gpio->write(GPIO_TEST_LED, _commanded_on);

    if (!_commanded_on) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (now - _last_poll_ms >= POLL_MS) {
        _last_poll_ms = now;
        service();
    }
    if (now - _last_print_ms >= PRINT_MS) {
        _last_print_ms = now;
        print_status();
    }
}

#endif // AP_PERIPH_SOLAR_BMS
