#include "AP_Periph.h"

#if AP_PERIPH_ABP2_PRESSURE_ENABLED

#include "abp2_pressure_backend.h"

extern const AP_HAL::HAL &hal;

// out-of-line definitions for the in-class constexpr arrays/members
constexpr uint8_t ABP2_Backend::MEASURE_CMD[3];

#ifndef ABP2_HEALTHY_TIMEOUT_MS
#define ABP2_HEALTHY_TIMEOUT_MS 500
#endif

ABP2_Backend::ABP2_Backend(ABP2Pressure &frontend, uint8_t instance,
                           const ABP2Pressure::SensorConfig &config) :
    _frontend(frontend),
    _instance(instance),
    _config(config)
{
}

/*
  Pressure transfer function (datasheet section 8.1.1):

    Pressure = (counts - OUT_MIN) / (OUT_MAX - OUT_MIN) * (Pmax - Pmin) + Pmin

  with OUT_MIN/OUT_MAX the 10%/90% points of the 24-bit output span. The result
  is returned in Pascals because Pmin/Pmax are configured in Pascals.
*/
float ABP2_Backend::convert_pressure(uint32_t counts) const
{
    return (((float)counts - OUT_MIN) / (OUT_MAX - OUT_MIN)) *
           (_config.p_max_pa - _config.p_min_pa) + _config.p_min_pa;
}

/*
  Temperature transfer function (datasheet section 8.1.2):

    Temperature = counts * (Tmax - Tmin) / (2^24 - 1) + Tmin,  Tmax=150, Tmin=-50
*/
float ABP2_Backend::convert_temperature(uint32_t counts)
{
    return ((float)counts * 200.0f / 16777215.0f) - 50.0f;
}

/*
  Decode a 7-byte reading and accumulate it. Runs in the device thread.
  Returns false without accumulating if the status byte reports busy/error.
*/
bool ABP2_Backend::process_reading(const uint8_t buf[7])
{
    const uint8_t status = buf[0];
    if (status & (STATUS_BUSY | STATUS_MEM_ERR)) {
        return false;
    }

    const uint32_t p_counts = ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
    const uint32_t t_counts = ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 8) | buf[6];

    const float p_pa = convert_pressure(p_counts);
    const float t_c  = convert_temperature(t_counts);

    WITH_SEMAPHORE(_sem);
    _pressure_sum    += p_pa;
    _temperature_sum += t_c;
    _press_count++;
    _temp_count++;
    _last_sample_ms = AP_HAL::millis();
    return true;
}

// main thread: drain the accumulator into the latest averaged values
void ABP2_Backend::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (_press_count > 0) {
        _pressure_pa  = _pressure_sum / _press_count;
        _pressure_sum = 0;
        _press_count  = 0;
    }
    if (_temp_count > 0) {
        _temperature_c  = _temperature_sum / _temp_count;
        _temperature_sum = 0;
        _temp_count      = 0;
    }
}

bool ABP2_Backend::healthy(void) const
{
    return _last_sample_ms != 0 &&
           (AP_HAL::millis() - _last_sample_ms) < ABP2_HEALTHY_TIMEOUT_MS;
}

#endif // AP_PERIPH_ABP2_PRESSURE_ENABLED
