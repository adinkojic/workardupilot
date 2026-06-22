#pragma once

#if AP_PERIPH_ABP2_PRESSURE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/Semaphores.h>

#include "abp2_pressure.h"

/*
  Backend base class for the Honeywell ABP2 pressure sensor.

  The wire protocol is identical for I2C and SPI: issue the 3-byte measurement
  command 0xAA 0x00 0x00, wait at least 5 ms for the conversion, then read 7
  bytes [status, P[23:16], P[15:8], P[7:0], T[23:16], T[15:8], T[7:0]]. The
  24-bit transfer functions (datasheet section 8.1) are therefore shared here;
  only the bus transactions differ between subclasses.

  Each subclass runs a small state machine in its own device-thread callback,
  accumulating samples behind a semaphore. update() (main thread) averages the
  accumulated samples into the latest pressure/temperature.
*/
class ABP2_Backend {
public:
    ABP2_Backend(ABP2Pressure &frontend, uint8_t instance,
                 const ABP2Pressure::SensorConfig &config);
    virtual ~ABP2_Backend() {}

    // probe and start the periodic callback; returns false if the device is absent
    virtual bool init(void) = 0;

    // main thread: average the accumulated samples into the latest values
    void update(void);

    float    pressure(void) const    { return _pressure_pa; }   // Pascals
    float    temperature(void) const { return _temperature_c; } // degrees C
    bool     healthy(void) const;
    uint8_t  instance(void) const    { return _instance; }
    const ABP2Pressure::SensorConfig &config(void) const { return _config; }

protected:
    // decode a 7-byte [status + 24-bit pressure + 24-bit temperature] reading;
    // returns false (and accumulates nothing) if the status byte is bad
    bool process_reading(const uint8_t buf[7]);

    ABP2Pressure &_frontend;
    const uint8_t _instance;
    const ABP2Pressure::SensorConfig _config;

    // 3-byte measurement command, shared by both bus types
    static constexpr uint8_t MEASURE_CMD[3] = { 0xAA, 0x00, 0x00 };

    // status byte bits (datasheet table 22/26)
    static constexpr uint8_t STATUS_BUSY    = (1U << 5); // 1 = conversion not yet complete
    static constexpr uint8_t STATUS_MEM_ERR = (1U << 2); // 1 = power-up integrity check failed

    static constexpr uint32_t CONV_US = 5000;            // >= 5 ms A/D conversion time

private:
    // 24-bit transfer functions (datasheet section 8.1)
    float convert_pressure(uint32_t counts) const;
    static float convert_temperature(uint32_t counts);

    HAL_Semaphore _sem;             // protects the accumulator

    // accumulator: written by the device thread, drained by update()
    float    _pressure_sum {0};
    float    _temperature_sum {0};
    uint32_t _press_count {0};
    uint32_t _temp_count {0};
    uint32_t _last_sample_ms {0};

    // latest averaged values (main thread only)
    float    _pressure_pa {0};
    float    _temperature_c {0};

    // 10% and 90% of 2^24 counts - the calibrated output span (datasheet 8.1.1)
    static constexpr float OUT_MIN = 1677722.0f;
    static constexpr float OUT_MAX = 15099494.0f;
};

/*
  I2C backend. The G4 I2C device thread runs the timer() state machine: it
  sends the measurement command, then reads the result after the conversion.
*/
class ABP2_I2C : public ABP2_Backend {
public:
    using ABP2_Backend::ABP2_Backend;
    ~ABP2_I2C(void) { delete _dev; }

    bool init(void) override;

private:
    void timer(void);

    AP_HAL::I2CDevice *_dev {nullptr};
    uint8_t  _state {0};        // 0 = send command, 1 = wait then read
    uint32_t _command_send_us {0};
};

/*
  SPI backend (mode 0). Implemented for completeness; the wire-level read uses a
  full-duplex transfer so the NOP byte 0xF0 is clocked out per datasheet 7.5.
*/
class ABP2_SPI : public ABP2_Backend {
public:
    using ABP2_Backend::ABP2_Backend;
    ~ABP2_SPI(void) { delete _dev; }

    bool init(void) override;

private:
    void timer(void);

    AP_HAL::SPIDevice *_dev {nullptr};
    uint8_t  _state {0};        // 0 = send command, 1 = wait then read
    uint32_t _command_send_us {0};
};

#endif // AP_PERIPH_ABP2_PRESSURE_ENABLED
