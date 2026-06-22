#pragma once

#if AP_PERIPH_ABP2_PRESSURE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

/*
  Honeywell ABP2 Series board-mount pressure sensor driver (front end).

  The ABP2 Series provides a 24-bit digital I2C or SPI output for compensated
  pressure and temperature (datasheet 32350268, Issue I). This front end mirrors
  the AP_Airspeed frontend/backend split: it owns one backend per configured
  sensor and exposes the averaged pressure (in Pascals) and temperature.

  Multiple sensors are configured from the hwdef.dat table HAL_PERIPH_ABP2_SENSORS,
  each with its own bus, address and pressure range. The pressure range is given
  in Pascals so that the same transfer function works for any part number; helper
  macros (ABP2_INH2O()/ABP2_PSI()/ABP2_BAR()/...) convert the datasheet units.
*/

// bus_type values
#define ABP2_BUS_I2C  0
#define ABP2_BUS_SPI  1

// type values (informational - reported alongside the reading)
#define ABP2_GAUGE        0
#define ABP2_ABSOLUTE     1
#define ABP2_DIFFERENTIAL 2

// pressure-range unit helpers: convert a datasheet full-scale value to Pascals
#define ABP2_PA(x)    ((float)(x))
#define ABP2_KPA(x)   ((float)(x) * 1000.0f)
#define ABP2_MBAR(x)  ((float)(x) * 100.0f)
#define ABP2_BAR(x)   ((float)(x) * 100000.0f)
#define ABP2_PSI(x)   ((float)(x) * 6894.757f)
#define ABP2_INH2O(x) ((float)(x) * 248.84f)   // inch of water @ 60 degF

#ifndef ABP2_MAX_SENSORS
#define ABP2_MAX_SENSORS 4
#endif

class ABP2_Backend;

class ABP2Pressure {
public:
    friend class ABP2_Backend;

    // per-sensor configuration, supplied as the HAL_PERIPH_ABP2_SENSORS table
    struct SensorConfig {
        uint8_t  bus_type;          // ABP2_BUS_I2C or ABP2_BUS_SPI
        uint8_t  bus;               // I2C bus number (ignored for SPI)
        uint8_t  address;           // I2C 7-bit address (ignored for SPI)
        float    p_min_pa;          // pressure at the 10% output count, in Pascals
        float    p_max_pa;          // pressure at the 90% output count, in Pascals
        uint8_t  type;              // ABP2_GAUGE / ABP2_ABSOLUTE / ABP2_DIFFERENTIAL
        const char *spi_device;     // SPIDEV name (ignored for I2C); omit for I2C entries
    };

    ABP2Pressure() {}
    ~ABP2Pressure();

    /* Do not allow copies */
    CLASS_NO_COPY(ABP2Pressure);

    // called from the main loop: refresh readings, broadcast and print in Pascals
    void update(void);

private:
    void init(void);

    ABP2_Backend *backends[ABP2_MAX_SENSORS] {};
    uint8_t num_sensors {0};
    bool    initialised {false};

    uint32_t last_bcast_ms {0};
    uint32_t last_print_ms {0};
};

#endif // AP_PERIPH_ABP2_PRESSURE_ENABLED
