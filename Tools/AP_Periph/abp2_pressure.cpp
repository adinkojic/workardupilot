/*
  Honeywell ABP2DANT015PG2A3XX pressure/temperature sensor driver
  I2C interface, 14-bit pressure (0–15 PSI gauge), 11-bit temperature

  I2C protocol (per Honeywell AN009206):
    1. Write 3-byte measurement command: 0xAA 0x00 0x00
    2. Wait ≥5 ms for A/D conversion
    3. Read 7 bytes: [status, P[23:16], P[15:8], P[7:0], T[23:16], T[15:8], T[7:0]]
       Status bit 2 = busy, bit 5 = memory error

  Architecture mirrors AP_Airspeed_MS5525:
    - I2C transactions run in a dedicated device-thread callback (timer())
    - timer() accumulates averaged samples behind a semaphore
    - update() (main loop) drains the accumulator and broadcasts via DroneCAN +
      can_printf
*/

#include "AP_Periph.h"

#if AP_PERIPH_ABP2_PRESSURE_ENABLED

#include "abp2_pressure.h"
#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

static constexpr uint8_t MEASURE_CMD[3] = {0xAA, 0x00, 0x00};
static constexpr uint8_t STATUS_BUSY    = (1U << 2);
static constexpr uint8_t STATUS_MEM_ERR = (1U << 5);

// ---------------------------------------------------------------------------
// init – probe device, set retries, register periodic callback
// ---------------------------------------------------------------------------
bool ABP2Pressure::init()
{
    dev = hal.i2c_mgr->get_device_ptr(I2C_BUS, I2C_ADDR);
    if (!dev) {
        can_printf("ABP2: no device on bus %u addr 0x%02X", I2C_BUS, I2C_ADDR);
        return false;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    // probe: send measure command and check for an ACK
    dev->set_retries(5);
    if (!dev->transfer(MEASURE_CMD, sizeof(MEASURE_CMD), nullptr, 0)) {
        can_printf("ABP2: probe failed");
        return false;
    }

    can_printf("ABP2: found on bus %u addr 0x%02X", I2C_BUS, I2C_ADDR);

    dev->set_retries(2);

    // register periodic callback – runs at TIMER_HZ in the I2C device thread
    dev->register_periodic_callback(1000000UL / TIMER_HZ,
                                    FUNCTOR_BIND_MEMBER(&ABP2Pressure::timer, void));
    return true;
}

// ---------------------------------------------------------------------------
// timer – runs in I2C device thread at TIMER_HZ
// ---------------------------------------------------------------------------
void ABP2Pressure::timer()
{
    if (state == 0) {
        // send measurement command
        if (dev->transfer(MEASURE_CMD, sizeof(MEASURE_CMD), nullptr, 0)) {
            command_send_us = AP_HAL::micros();
            state = 1;
        }
        return;
    }

    // state == 1: wait for conversion, then read
    if (AP_HAL::micros() - command_send_us < CONV_US) {
        return;
    }

    uint8_t buf[7] {};
    if (!dev->transfer(nullptr, 0, buf, sizeof(buf))) {
        state = 0;
        return;
    }

    if (buf[0] & STATUS_BUSY) {
        // sensor still converting – leave state = 1 and retry next tick
        return;
    }
    if (buf[0] & STATUS_MEM_ERR) {
        state = 0;
        return;
    }

    calculate(buf);
    state = 0;
}

// ---------------------------------------------------------------------------
// calculate – convert raw bytes and accumulate (called from timer thread)
// ---------------------------------------------------------------------------
void ABP2Pressure::calculate(const uint8_t buf[7])
{
    // 14-bit pressure: bits [23:10] of the 24-bit bridge word
    const uint32_t p_raw    = ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
    const uint32_t p_counts = (p_raw >> 10) & 0x3FFF;

    // 11-bit temperature: bits [23:13] of the 24-bit temperature word
    const uint32_t t_raw    = ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 8) | buf[6];
    const uint32_t t_counts = (t_raw >> 13) & 0x07FF;

    const float p_pa = ((float)(p_counts - OUT_MIN) / (float)(OUT_MAX - OUT_MIN))
                       * PMAX_PSI * PSI_TO_PA;
    const float t_c  = ((float)t_counts * 200.0f / 2047.0f) - 50.0f;

    WITH_SEMAPHORE(sem);
    pressure_sum    += p_pa;
    temperature_sum += t_c;
    press_count++;
    temp_count++;
    last_sample_ms = AP_HAL::millis();
}

// ---------------------------------------------------------------------------
// update – called from main loop; averages samples and broadcasts over CAN
// ---------------------------------------------------------------------------
void ABP2Pressure::update()
{
    static bool inited = false;
    if (!inited) {
        inited = true;
        if (!init()) {
            return;
        }
    }

    if (!dev) {
        return;
    }

    // drain accumulator
    {
        WITH_SEMAPHORE(sem);
        if (press_count > 0) {
            pressure_pa    = pressure_sum    / press_count;
            pressure_sum   = 0.0f;
            press_count    = 0;
        }
        if (temp_count > 0) {
            temperature_c   = temperature_sum / temp_count;
            temperature_sum = 0.0f;
            temp_count      = 0;
        }
    }

    // stale-data guard (matches MS5525 pattern)
    if ((AP_HAL::millis() - last_sample_ms) > 500) {
        return;
    }

    const uint32_t now = AP_HAL::millis();

    // broadcast DroneCAN at BCAST_MS rate
    if (now - last_bcast_ms >= BCAST_MS) {
        last_bcast_ms = now;

        {
            uavcan_equipment_air_data_StaticPressure pkt {};
            pkt.static_pressure          = pressure_pa;
            pkt.static_pressure_variance = 0;
            uint8_t  buf[UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_MAX_SIZE];
            uint16_t len = uavcan_equipment_air_data_StaticPressure_encode(
                               &pkt, buf, !periph.canfdout());
            periph.canard_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_SIGNATURE,
                                    UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_ID,
                                    CANARD_TRANSFER_PRIORITY_LOW,
                                    buf, len);
        }
        {
            uavcan_equipment_air_data_StaticTemperature pkt {};
            pkt.static_temperature          = C_TO_KELVIN(temperature_c);
            pkt.static_temperature_variance = 0;
            uint8_t  buf[UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_MAX_SIZE];
            uint16_t len = uavcan_equipment_air_data_StaticTemperature_encode(
                               &pkt, buf, !periph.canfdout());
            periph.canard_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_SIGNATURE,
                                    UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_ID,
                                    CANARD_TRANSFER_PRIORITY_LOW,
                                    buf, len);
        }
    }

    // log to CAN bus at PRINT_MS rate
    if (now - last_print_ms >= PRINT_MS) {
        last_print_ms = now;
        can_printf("ABP2: P=%.0f Pa (%.2f PSI) T=%.1f C",
                   (double)pressure_pa,
                   (double)(pressure_pa / PSI_TO_PA),
                   (double)temperature_c);
    }
}

#endif // AP_PERIPH_ABP2_PRESSURE_ENABLED
