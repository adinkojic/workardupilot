/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  backend driver for the Honeywell ABP2 series I2C pressure sensor.

  The sensor full-scale range is set with the ARSPD_PSI_RANGE parameter, in
  PSI: a +/-1 psi differential part (e.g. ABP2DDAN001PDSA3) uses 1.0, a
  +/-25 mbar part 0.363, etc. The I2C bus is set with ARSPD_BUS.
 */

#include "AP_Airspeed_ABP2.h"

#if AP_AIRSPEED_ABP2_ENABLED

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

// 3-byte single measurement command (datasheet section 7.3)
static const uint8_t ABP2_MEASURE_CMD[3] = { 0xAA, 0x00, 0x00 };

#define ABP2_TIMER_HZ 100

// PSI to Pascal conversion
#define ABP2_PSI_TO_PA 6894.757f

bool ABP2_Pressure_sensor::measure()
{
    return dev->transfer(ABP2_MEASURE_CMD, sizeof(ABP2_MEASURE_CMD), nullptr, 0);
}

ABP2_Pressure_sensor::Status ABP2_Pressure_sensor::collect(float &pressure_pa, float &temperature)
{
    uint8_t raw[7];
    if (!dev->transfer(nullptr, 0, raw, sizeof(raw))) {
        return Status::Fault;
    }

    const uint8_t status = raw[0];
    // always-zero bits set or power bit clear means this is not a valid
    // ABP2 status byte
    if ((status & STATUS_ZERO_BITS) != 0 || (status & STATUS_POWER) == 0) {
        return Status::Fault;
    }
    if (status & STATUS_BUSY) {
        return Status::Busy;
    }
    if (status & (STATUS_MEM_ERR | STATUS_MATH_SAT)) {
        return Status::Fault;
    }

    const uint32_t p_counts = ((uint32_t)raw[1] << 16) | ((uint32_t)raw[2] << 8) | raw[3];
    const uint32_t t_counts = ((uint32_t)raw[4] << 16) | ((uint32_t)raw[5] << 8) | raw[6];

    // pressure transfer function (datasheet section 8.1.1)
    pressure_pa = (((float)p_counts - OUT_MIN) / (OUT_MAX - OUT_MIN)) *
                  (p_max_pa - p_min_pa) + p_min_pa;

    // temperature transfer function (datasheet section 8.1.2)
    temperature = ((float)t_counts * 200.0f / 16777215.0f) - 50.0f;

    return Status::Normal;
}

// probe and initialise the sensor
bool AP_Airspeed_ABP2::init()
{
    // differential parts are symmetric about zero; ARSPD_PSI_RANGE gives the
    // full-scale magnitude
    const float range_pa = get_psi_range() * ABP2_PSI_TO_PA;
    sensor.set_range(-range_pa, range_pa);

    _dev = hal.i2c_mgr->get_device_ptr(get_bus(), HAL_AIRSPEED_ABP2_I2C_ADDR);
    if (!_dev) {
        return false;
    }

    {
        WITH_SEMAPHORE(_dev->get_semaphore());

        _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
        _dev->set_retries(5);

        // probe: start a measurement and check we get a valid status byte back
        if (!sensor.measure()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ABP2[%u]: no sensor found", get_instance());
            return false;
        }
        hal.scheduler->delay(10);

        float pressure, temperature;
        if (sensor.collect(pressure, temperature) != ABP2_Pressure_sensor::Status::Normal) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ABP2[%u]: probe read failed", get_instance());
            return false;
        }

        _dev->set_retries(2);
    }

    _dev->set_device_type(uint8_t(DevType::ABP2));
    set_bus_id(_dev->get_bus_id());

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ABP2[%u]: Found bus %u addr 0x%02x",
                  get_instance(), _dev->bus_num(), _dev->get_bus_address());

    _dev->register_periodic_callback(1000000UL / ABP2_TIMER_HZ,
                                     FUNCTOR_BIND_MEMBER(&AP_Airspeed_ABP2::timer, void));
    return true;
}

// runs in the I2C device thread at ABP2_TIMER_HZ; alternating ticks start a
// measurement and collect the result, so each read comes >= 10ms (well over
// the ~5ms conversion time) after the command
void AP_Airspeed_ABP2::timer()
{
    if (measurement_requested) {
        float pressure_pa, temperature;
        switch (sensor.collect(pressure_pa, temperature)) {
        case ABP2_Pressure_sensor::Status::Normal: {
            WITH_SEMAPHORE(sem);
            press_sum += pressure_pa;
            temp_sum += temperature;
            press_count++;
            temp_count++;
            last_sample_ms = AP_HAL::millis();
            break;
        }
        case ABP2_Pressure_sensor::Status::Busy:
            // conversion not finished, read again on the next tick
            return;
        case ABP2_Pressure_sensor::Status::Fault:
            break;
        }
    }

    measurement_requested = sensor.measure();
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_ABP2::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(sem);

    if (AP_HAL::millis() - last_sample_ms > 100) {
        return false;
    }

    if (press_count == 0) {
        pressure = last_pressure;
        return true;
    }

    last_pressure = pressure = press_sum / press_count;
    press_count = 0;
    press_sum = 0;

    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_ABP2::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);

    if (AP_HAL::millis() - last_sample_ms > 100) {
        return false;
    }

    if (temp_count == 0) {
        temperature = last_temperature;
        return true;
    }

    last_temperature = temperature = temp_sum / temp_count;
    temp_count = 0;
    temp_sum = 0;

    return true;
}

#endif  // AP_AIRSPEED_ABP2_ENABLED
