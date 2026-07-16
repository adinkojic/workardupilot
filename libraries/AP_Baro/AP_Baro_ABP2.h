#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_ABP2_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>

// Baro uses the ABP2 pressure sensor class from airspeed, airspeed must be enabled
#include <AP_Airspeed/AP_Airspeed_config.h>
#if !AP_AIRSPEED_ABP2_ENABLED
#error ABP2 Baro requires ABP2 Airspeed
#endif

#include <AP_Airspeed/AP_Airspeed_ABP2.h>

#ifndef HAL_BARO_ABP2_I2C_ADDR
 #define HAL_BARO_ABP2_I2C_ADDR 0x28
#endif

// absolute pressure range of the fitted part, in Pascals. The default suits
// a 0 to 15 psi absolute part (e.g. ABP2LANT015PA2A3); override in hwdef.dat
// for other part numbers
#ifndef HAL_BARO_ABP2_PMIN_PA
 #define HAL_BARO_ABP2_PMIN_PA 0.0f
#endif
#ifndef HAL_BARO_ABP2_PMAX_PA
 #define HAL_BARO_ABP2_PMAX_PA (15.0f * 6894.757f)
#endif

class AP_Baro_ABP2 : public AP_Baro_Backend {
public:
    AP_Baro_ABP2(AP_Baro &baro, AP_HAL::Device *dev);

    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);

protected:
    bool init();

    void timer();

    AP_HAL::Device *dev;

    ABP2_Pressure_sensor sensor{dev};

    uint8_t instance;

    uint32_t count;
    float pressure_sum;
    float temperature_sum;

    bool measurement_requested;
};

#endif  // AP_BARO_ABP2_ENABLED
