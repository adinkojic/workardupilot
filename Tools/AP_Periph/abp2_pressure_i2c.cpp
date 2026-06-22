#include "AP_Periph.h"

#if AP_PERIPH_ABP2_PRESSURE_ENABLED

#include "abp2_pressure_backend.h"

extern const AP_HAL::HAL &hal;

#ifndef ABP2_TIMER_HZ
#define ABP2_TIMER_HZ 100   // device-thread callback rate
#endif

bool ABP2_I2C::init(void)
{
    _dev = hal.i2c_mgr->get_device_ptr(_config.bus, _config.address);
    if (_dev == nullptr) {
        can_printf("ABP2[%u]: no I2C device on bus %u addr 0x%02X",
                   _instance, _config.bus, _config.address);
        return false;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    // probe: a bare measurement command must be ACKed
    _dev->set_retries(5);
    if (!_dev->transfer(MEASURE_CMD, sizeof(MEASURE_CMD), nullptr, 0)) {
        can_printf("ABP2[%u]: probe failed on bus %u addr 0x%02X",
                   _instance, _config.bus, _config.address);
        delete _dev;
        _dev = nullptr;
        return false;
    }

    can_printf("ABP2[%u]: found on I2C bus %u addr 0x%02X",
               _instance, _config.bus, _config.address);

    _dev->set_retries(2);
    _dev->register_periodic_callback(1000000UL / ABP2_TIMER_HZ,
                                     FUNCTOR_BIND_MEMBER(&ABP2_I2C::timer, void));
    return true;
}

// runs in the I2C device thread at ABP2_TIMER_HZ
void ABP2_I2C::timer(void)
{
    if (_state == 0) {
        // send the measurement command, then read on a later tick
        if (_dev->transfer(MEASURE_CMD, sizeof(MEASURE_CMD), nullptr, 0)) {
            _command_send_us = AP_HAL::micros();
            _state = 1;
        }
        return;
    }

    // state 1: wait for the conversion, then read status + pressure + temperature
    if (AP_HAL::micros() - _command_send_us < CONV_US) {
        return;
    }

    uint8_t buf[7] {};
    if (!_dev->transfer(nullptr, 0, buf, sizeof(buf))) {
        _state = 0;
        return;
    }

    // if the sensor still reports busy, retry the read on the next tick
    if (buf[0] & STATUS_BUSY) {
        return;
    }

    process_reading(buf);
    _state = 0;
}

#endif // AP_PERIPH_ABP2_PRESSURE_ENABLED
