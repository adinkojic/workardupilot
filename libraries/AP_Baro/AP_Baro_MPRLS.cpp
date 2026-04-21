/*
  Honeywell MPRLS0025PA00001A barometer driver
  24-bit absolute pressure (0–25 PSI), SPI interface, no temperature output

  SPI protocol (per Honeywell TN_008202-3):
    1. Assert CS, send {0xAA, 0x00, 0x00}, deassert CS  — start measurement
    2. Wait ≥5 ms for A/D conversion
    3. Assert CS, read 4 bytes, deassert CS:
         buf[0] = status  (bit 6: busy, bit 5: memory err, bit 2: math sat)
         buf[1] = P[23:16]
         buf[2] = P[15:8]
         buf[3] = P[7:0]

  Architecture mirrors AP_Airspeed_MS5525:
    - register_periodic_callback drives a two-state machine in _timer()
    - _calculate() accumulates samples behind _sem
    - update() averages the accumulator and calls _copy_to_frontend()
*/

#include "AP_Baro_MPRLS.h"

#if AP_BARO_MPRLS_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

static const uint8_t MEASURE_CMD[3] = {0xAA, 0x00, 0x00};

AP_Baro_MPRLS::AP_Baro_MPRLS(AP_Baro &baro, AP_HAL::Device &dev)
    : AP_Baro_Backend(baro)
    , _dev(&dev)
{}

AP_Baro_Backend *AP_Baro_MPRLS::probe(AP_Baro &baro, AP_HAL::Device &dev)
{
    AP_Baro_MPRLS *sensor = NEW_NOTHROW AP_Baro_MPRLS(baro, dev);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_MPRLS::_init()
{
    if (!_dev) {
        return false;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(5);

    // probe: send measure command and verify the device ACKs
    if (!_dev->transfer(MEASURE_CMD, sizeof(MEASURE_CMD), nullptr, 0)) {
        return false;
    }

    // give one conversion cycle and check we can read a plausible status byte
    hal.scheduler->delay(10);
    uint8_t buf[4] {};
    if (!_dev->transfer(nullptr, 0, buf, sizeof(buf))) {
        return false;
    }
    // busy or memory-error on fresh probe means something is wrong
    if (buf[0] & STATUS_MEM_ERR) {
        return false;
    }

    _instance = _frontend.register_sensor();

    _dev->set_device_type(uint8_t(DEVTYPE_BARO_MPRLS));
    set_bus_id(_instance, _dev->get_bus_id());

    _dev->set_retries(2);

    _dev->register_periodic_callback(1000000UL / TIMER_HZ,
                                     FUNCTOR_BIND_MEMBER(&AP_Baro_MPRLS::_timer, void));
    return true;
}

// ---------------------------------------------------------------------------
// _timer – runs in SPI device thread at TIMER_HZ
// ---------------------------------------------------------------------------
void AP_Baro_MPRLS::_timer()
{
    if (_state == 0) {
        if (_dev->transfer(MEASURE_CMD, sizeof(MEASURE_CMD), nullptr, 0)) {
            _cmd_sent_us = AP_HAL::micros();
            _state = 1;
        }
        return;
    }

    // state == 1: wait for conversion, then read
    if (AP_HAL::micros() - _cmd_sent_us < CONV_US) {
        return;
    }

    uint8_t buf[4] {};
    if (!_dev->transfer(nullptr, 0, buf, sizeof(buf))) {
        _state = 0;
        return;
    }

    if (buf[0] & STATUS_BUSY) {
        // still converting – retry next tick, stay in state 1
        return;
    }
    if (buf[0] & (STATUS_MEM_ERR | STATUS_MATH_SAT)) {
        _state = 0;
        return;
    }

    _calculate(buf);
    _state = 0;
}

// ---------------------------------------------------------------------------
// _calculate – convert raw bytes and accumulate (called from timer thread)
// ---------------------------------------------------------------------------
void AP_Baro_MPRLS::_calculate(const uint8_t buf[4])
{
    const uint32_t raw = ((uint32_t)buf[1] << 16)
                       | ((uint32_t)buf[2] << 8)
                       |  (uint32_t)buf[3];

    if (raw < OUT_MIN || raw > OUT_MAX) {
        return;
    }

    const float p_pa = ((float)(raw - OUT_MIN) / (float)(OUT_MAX - OUT_MIN))
                       * PMAX_PSI * PSI_TO_PA;

    if (!pressure_ok(p_pa)) {
        return;
    }

    WITH_SEMAPHORE(_sem);
    _pressure_sum += p_pa;
    _press_count++;
    _last_sample_ms = AP_HAL::millis();
}

// ---------------------------------------------------------------------------
// update – called by AP_Baro::update() on the main thread
// ---------------------------------------------------------------------------
void AP_Baro_MPRLS::update()
{
    WITH_SEMAPHORE(_sem);

    if (_press_count == 0) {
        return;
    }

    _pressure_pa  = _pressure_sum / _press_count;
    _pressure_sum = 0.0f;
    _press_count  = 0;

    // MPRLS has no temperature sensor; report a fixed 25 °C placeholder
    _copy_to_frontend(_instance, _pressure_pa, 25.0f);
}

#endif // AP_BARO_MPRLS_ENABLED
