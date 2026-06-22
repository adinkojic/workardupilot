/*
  Maxim MAX31875 I2C temperature sensor driver

  I2C protocol:
    - Register 0x00: temperature (read 2 bytes, MSB first)
      bits [15:4] = signed 12-bit value, LSB = 0.0625 °C
    - Register 0x01: configuration (default 0x0000 = continuous, 12-bit)
    - No init write needed; sensor starts converting on power-on.

  Architecture mirrors ABP2Pressure:
    - Background device-thread callback (timer()) reads at TIMER_HZ
    - Accumulates samples behind a semaphore
    - update() (main loop) drains accumulator and prints via can_printf
*/

#include "AP_Periph.h"

#if AP_PERIPH_MAX31875_ENABLED

#include "max31875_temp.h"

extern const AP_HAL::HAL &hal;

static constexpr uint8_t REG_TEMP = 0x00;

bool MAX31875Temp::init()
{
    dev = hal.i2c_mgr->get_device_ptr(i2c_bus, i2c_addr);
    if (!dev) {
        can_printf("MAX31875: no device on bus %u addr 0x%02X", i2c_bus, i2c_addr);
        return false;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    dev->set_retries(5);

    // Probe: read temperature register and expect an ACK
    uint8_t buf[2];
    if (!dev->read_registers(REG_TEMP, buf, sizeof(buf))) {
        can_printf("MAX31875: probe failed on bus %u addr 0x%02X", i2c_bus, i2c_addr);
        return false;
    }

    can_printf("MAX31875: found on bus %u addr 0x%02X", i2c_bus, i2c_addr);

    dev->set_retries(2);

    dev->register_periodic_callback(1000000UL / TIMER_HZ,
                                    FUNCTOR_BIND_MEMBER(&MAX31875Temp::timer, void));
    return true;
}

// Runs in the I2C device thread at TIMER_HZ
void MAX31875Temp::timer()
{
    uint8_t buf[2];
    if (!dev->read_registers(REG_TEMP, buf, sizeof(buf))) {
        return;
    }

    // bits [15:4] are the signed 12-bit temperature; LSB = 0.0625 °C
    const int16_t raw = (int16_t)((uint16_t(buf[0]) << 8) | buf[1]);
    const float t_c   = (raw >> 4) * 0.0625f;

    WITH_SEMAPHORE(sem);
    temperature_sum += t_c;
    temp_count++;
    last_sample_ms = AP_HAL::millis();
}

// Called from the main loop
void MAX31875Temp::update()
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

    {
        WITH_SEMAPHORE(sem);
        if (temp_count > 0) {
            temperature_c   = temperature_sum / temp_count;
            temperature_sum = 0.0f;
            temp_count      = 0;
        }
    }

    // Stale-data guard: sensor should deliver at TIMER_HZ
    if ((AP_HAL::millis() - last_sample_ms) > 2000) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (now - last_print_ms >= PRINT_MS) {
        last_print_ms = now;
        hal.console->printf("MAX31875: %.2f C\n", (double)temperature_c);
    }
}

#endif // AP_PERIPH_MAX31875_ENABLED
