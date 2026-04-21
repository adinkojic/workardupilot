#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_MPRLS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>

/*
  Honeywell MPRLS0025PA00001A
  0–25 PSI absolute, 24-bit SPI output, no temperature output
  SPI Mode 0, max 800 kHz
*/
class AP_Baro_MPRLS : public AP_Baro_Backend
{
public:
    AP_Baro_MPRLS(AP_Baro &baro, AP_HAL::Device &dev);

    ~AP_Baro_MPRLS() { delete _dev; }

    // probe and initialise the sensor; returns nullptr on failure
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);

    // push accumulated data to frontend (called by AP_Baro::update)
    void update() override;

private:
    bool _init();

    // device-thread callback
    void _timer();

    // convert raw 4-byte buffer and accumulate
    void _calculate(const uint8_t buf[4]);

    AP_HAL::Device *_dev;
    uint8_t         _instance;

    // accumulator (written by _timer, read by update, both under _sem)
    float    _pressure_sum  = 0.0f;
    uint32_t _press_count   = 0;
    float    _pressure_pa   = 0.0f;
    uint32_t _last_sample_ms = 0;

    uint8_t  _state          = 0;   // 0 = send command, 1 = read data
    uint32_t _cmd_sent_us    = 0;

    // MPRLS output range: 10% and 90% of 2^24
    static constexpr uint32_t OUT_MIN    = 1677722UL;
    static constexpr uint32_t OUT_MAX    = 15099494UL;
    static constexpr float    PMAX_PSI   = 25.0f;
    static constexpr float    PSI_TO_PA  = 6894.757f;
    static constexpr uint32_t CONV_US    = 5000;      // 5 ms conversion time
    static constexpr uint32_t TIMER_HZ   = 200;

    // status byte masks
    static constexpr uint8_t STATUS_BUSY    = (1U << 6);
    static constexpr uint8_t STATUS_MEM_ERR = (1U << 5);
    static constexpr uint8_t STATUS_MATH_SAT= (1U << 2);
};

#endif // AP_BARO_MPRLS_ENABLED
