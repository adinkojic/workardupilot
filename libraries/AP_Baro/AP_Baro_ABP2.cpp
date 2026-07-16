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
  Honeywell ABP2 series absolute pressure sensor as a barometer, probed via
  the BARO_PROBE_EXT parameter. The pressure range defaults to a 0-15 psi
  absolute part; set HAL_BARO_ABP2_PMIN_PA/HAL_BARO_ABP2_PMAX_PA in hwdef.dat
  for other part numbers.
 */

#include "AP_Baro_ABP2.h"

#if AP_BARO_ABP2_ENABLED

extern const AP_HAL::HAL &hal;

#define ABP2_BARO_TIMER_HZ 100

AP_Baro_ABP2::AP_Baro_ABP2(AP_Baro &baro, AP_HAL::Device *_dev)
    : AP_Baro_Backend(baro)
    , dev(_dev)
{
}

AP_Baro_Backend *AP_Baro_ABP2::probe(AP_Baro &baro, AP_HAL::Device &_dev)
{
    AP_Baro_ABP2 *sensor = NEW_NOTHROW AP_Baro_ABP2(baro, &_dev);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_ABP2::init()
{
    if (!dev) {
        return false;
    }

    sensor.set_range(HAL_BARO_ABP2_PMIN_PA, HAL_BARO_ABP2_PMAX_PA);

    {
        WITH_SEMAPHORE(dev->get_semaphore());

        dev->set_retries(5);

        // probe: start a measurement and check we get a valid status byte back
        if (!sensor.measure()) {
            return false;
        }
        hal.scheduler->delay(10);

        float pressure, temperature;
        if (sensor.collect(pressure, temperature) != ABP2_Pressure_sensor::Status::Normal) {
            return false;
        }

        dev->set_retries(2);
    }

    instance = _frontend.register_sensor();
    dev->set_device_type(DEVTYPE_BARO_ABP2);
    set_bus_id(instance, dev->get_bus_id());

    dev->register_periodic_callback(1000000UL / ABP2_BARO_TIMER_HZ,
                                    FUNCTOR_BIND_MEMBER(&AP_Baro_ABP2::timer, void));
    return true;
}

// runs in the I2C device thread; alternating ticks start a measurement and
// collect the result (>= 10ms after the command, well over the ~5ms
// conversion time)
void AP_Baro_ABP2::timer()
{
    if (measurement_requested) {
        float pressure_pa, temperature;
        switch (sensor.collect(pressure_pa, temperature)) {
        case ABP2_Pressure_sensor::Status::Normal:
            if (pressure_ok(pressure_pa)) {
                WITH_SEMAPHORE(_sem);
                pressure_sum += pressure_pa;
                temperature_sum += temperature;
                count++;
            }
            break;

        case ABP2_Pressure_sensor::Status::Busy:
            // conversion not finished, read again on the next tick
            return;

        case ABP2_Pressure_sensor::Status::Fault:
            break;
        }
    }

    measurement_requested = sensor.measure();
}

// transfer data to the frontend
void AP_Baro_ABP2::update()
{
    WITH_SEMAPHORE(_sem);

    if (count == 0) {
        return;
    }

    _copy_to_frontend(instance, pressure_sum/count, temperature_sum/count);

    pressure_sum = 0;
    temperature_sum = 0;
    count = 0;
}

#endif  // AP_BARO_ABP2_ENABLED
