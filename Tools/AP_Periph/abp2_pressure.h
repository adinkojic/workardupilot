#pragma once

#if AP_PERIPH_ABP2_PRESSURE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

/*
  Honeywell ABP2DANT015PG2A3XX
  14-bit gauge pressure (0–15 PSI), 11-bit temperature
  I2C address 0x28, 3.3 V supply
*/
class ABP2Pressure {
public:
    friend class AP_Periph_FW;

    // called from main loop: average accumulated samples and broadcast over CAN
    void update(void);

    ~ABP2Pressure() { delete dev; }

private:
    bool init(void);

    // I2C device thread callback – runs at TIMER_HZ
    void timer(void);

    // convert raw 7-byte read buffer into pressure/temperature and accumulate
    void calculate(const uint8_t buf[7]);

    AP_HAL::I2CDevice *dev = nullptr;
    HAL_Semaphore      sem;

    // accumulated samples (written by timer, read by update under sem)
    float    pressure_sum    = 0.0f;
    float    temperature_sum = 0.0f;
    uint32_t press_count     = 0;
    uint32_t temp_count      = 0;

    // latest averaged values (main thread only)
    float    pressure_pa     = 0.0f;
    float    temperature_c   = 0.0f;

    uint32_t last_sample_ms  = 0;   // last time calculate() ran (set under sem)
    uint32_t last_bcast_ms   = 0;
    uint32_t last_print_ms   = 0;

    uint8_t  state           = 0;   // 0 = send command, 1 = read data
    uint32_t command_send_us = 0;

    static constexpr uint8_t  I2C_BUS    = 0;
    static constexpr uint8_t  I2C_ADDR   = 0x28;
    static constexpr uint32_t TIMER_HZ   = 200;          // callback rate
    static constexpr uint32_t CONV_US    = 5000;         // 5 ms conversion
    static constexpr uint32_t BCAST_MS   = 100;          // 10 Hz DroneCAN
    static constexpr uint32_t PRINT_MS   = 1000;         // 1 Hz can_printf
    static constexpr float    PMAX_PSI   = 15.0f;
    static constexpr float    PSI_TO_PA  = 6894.757f;
    static constexpr uint32_t OUT_MIN    = 1638;         // 10% of 2^14
    static constexpr uint32_t OUT_MAX    = 14745;        // 90% of 2^14
};

#endif // AP_PERIPH_ABP2_PRESSURE_ENABLED
