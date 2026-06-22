#pragma once

#if AP_PERIPH_MAX31875_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

/*
  Maxim MAX31875 ±0.5°C-accurate I2C temperature sensor
  I2C address 0x48–0x4F (A2/A1/A0 straps), default 0x48
  Register 0x00: 16-bit 2's-complement temperature, 12-bit resolution
    bits [15:4] = temp data, LSB = 0.0625 °C, bits [3:0] = 0
*/
class MAX31875Temp {
public:
    friend class AP_Periph_FW;

    MAX31875Temp(uint8_t bus, uint8_t addr) : i2c_bus(bus), i2c_addr(addr) {}

    void update(void);

    ~MAX31875Temp() { delete dev; }

private:
    bool init(void);
    void timer(void);

    AP_HAL::I2CDevice *dev = nullptr;
    HAL_Semaphore      sem;

    float    temperature_sum = 0.0f;
    uint32_t temp_count      = 0;
    float    temperature_c   = 0.0f;

    uint32_t last_sample_ms  = 0;
    uint32_t last_print_ms   = 0;

    const uint8_t i2c_bus;
    const uint8_t i2c_addr;

    static constexpr uint32_t TIMER_HZ = 4;
    static constexpr uint32_t PRINT_MS = 200;
};

#endif // AP_PERIPH_MAX31875_ENABLED
