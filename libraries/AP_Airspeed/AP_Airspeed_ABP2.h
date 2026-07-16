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
#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ABP2_ENABLED

/*
  backend driver for the Honeywell ABP2 series I2C pressure sensor
  (datasheet 32350268). Differential parts are used as airspeed sensors;
  the same wire protocol is reused by AP_Baro_ABP2 for absolute parts.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#include "AP_Airspeed_Backend.h"

#ifndef HAL_AIRSPEED_ABP2_I2C_ADDR
#define HAL_AIRSPEED_ABP2_I2C_ADDR 0x28
#endif

/*
  wire protocol and transfer functions shared between the airspeed and baro
  backends. Issue the 3-byte measurement command 0xAA 0x00 0x00, wait for the
  ~5ms conversion, then read 7 bytes:
    [status, P[23:16], P[15:8], P[7:0], T[23:16], T[15:8], T[7:0]]
 */
class ABP2_Pressure_sensor
{
public:
    ABP2_Pressure_sensor(AP_HAL::Device *&_dev) : dev(_dev) {}

    // pressure at the 10% and 90% output counts, in Pascals. Differential
    // parts use a symmetric range (-FS .. +FS), absolute parts 0 .. FS
    void set_range(float _p_min_pa, float _p_max_pa) {
        p_min_pa = _p_min_pa;
        p_max_pa = _p_max_pa;
    }

    // start a measurement, return true if the command was ACKed
    bool measure() WARN_IF_UNUSED;

    enum class Status {
        Normal, // valid reading returned
        Busy,   // conversion not finished, try again later
        Fault,  // bus error or bad status byte
    };

    // read the result of the last measurement: pressure in Pascals,
    // temperature in degrees C
    Status collect(float &pressure_pa, float &temperature) WARN_IF_UNUSED;

private:
    AP_HAL::Device *&dev;

    float p_min_pa;
    float p_max_pa;

    // status byte bits (datasheet table 22/26); bits 7,4,3,1 always read 0
    static constexpr uint8_t STATUS_POWER     = 1U << 6;
    static constexpr uint8_t STATUS_BUSY      = 1U << 5;
    static constexpr uint8_t STATUS_MEM_ERR   = 1U << 2;
    static constexpr uint8_t STATUS_MATH_SAT  = 1U << 0;
    static constexpr uint8_t STATUS_ZERO_BITS = 0x9A;

    // 10% and 90% of 2^24 counts - the calibrated output span (datasheet 8.1.1)
    static constexpr float OUT_MIN = 1677722.0f;
    static constexpr float OUT_MAX = 15099494.0f;
};

class AP_Airspeed_ABP2 : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_ABP2(AP_Airspeed &_frontend, uint8_t _instance) :
        AP_Airspeed_Backend(_frontend, _instance) {}
    ~AP_Airspeed_ABP2(void) {
        delete _dev;
    }

    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;

private:
    void timer();

    AP_HAL::Device *_dev;

    ABP2_Pressure_sensor sensor{_dev};

    bool measurement_requested;

    uint32_t last_sample_ms;
    float press_sum;
    float temp_sum;
    uint32_t press_count;
    uint32_t temp_count;
    float last_pressure;
    float last_temperature;
};

#endif  // AP_AIRSPEED_ABP2_ENABLED
