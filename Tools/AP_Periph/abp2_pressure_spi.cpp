#include "AP_Periph.h"

#if AP_PERIPH_ABP2_PRESSURE_ENABLED

#include "abp2_pressure_backend.h"

extern const AP_HAL::HAL &hal;

#ifndef ABP2_TIMER_HZ
#define ABP2_TIMER_HZ 100   // device-thread callback rate
#endif

// NOP command 0xF0 followed by dummy bytes, clocked out on MOSI while the
// status + pressure + temperature bytes are clocked in on MISO (datasheet 7.5).
static const uint8_t NOP_READ[7] = { 0xF0, 0, 0, 0, 0, 0, 0 };

bool ABP2_SPI::init(void)
{
    if (_config.spi_device == nullptr) {
        can_printf("ABP2[%u]: no SPI device name configured", _instance);
        return false;
    }

    _dev = hal.spi->get_device_ptr(_config.spi_device);
    if (_dev == nullptr) {
        can_printf("ABP2[%u]: no SPI device '%s'", _instance, _config.spi_device);
        return false;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    // datasheet 7.1: bring SS low briefly before the very first transaction.
    // A bare command transfer also serves as the probe.
    if (!_dev->transfer(MEASURE_CMD, sizeof(MEASURE_CMD), nullptr, 0)) {
        can_printf("ABP2[%u]: SPI probe failed on '%s'", _instance, _config.spi_device);
        delete _dev;
        _dev = nullptr;
        return false;
    }

    can_printf("ABP2[%u]: found on SPI '%s'", _instance, _config.spi_device);

    _dev->register_periodic_callback(1000000UL / ABP2_TIMER_HZ,
                                     FUNCTOR_BIND_MEMBER(&ABP2_SPI::timer, void));
    return true;
}

// runs in the SPI device thread at ABP2_TIMER_HZ
void ABP2_SPI::timer(void)
{
    if (_state == 0) {
        if (_dev->transfer(MEASURE_CMD, sizeof(MEASURE_CMD), nullptr, 0)) {
            _command_send_us = AP_HAL::micros();
            _state = 1;
        }
        return;
    }

    // state 1: wait for the conversion, then clock out 7 bytes full-duplex
    if (AP_HAL::micros() - _command_send_us < CONV_US) {
        return;
    }

    uint8_t buf[7] {};
    if (!_dev->transfer_fullduplex(NOP_READ, buf, sizeof(buf))) {
        _state = 0;
        return;
    }

    if (buf[0] & STATUS_BUSY) {
        return;
    }

    process_reading(buf);
    _state = 0;
}

#endif // AP_PERIPH_ABP2_PRESSURE_ENABLED
