/*
  Honeywell ABP2 Series board-mount pressure sensor support for AP_Periph.

  This is the front end: it builds one backend per entry of the hwdef.dat table
  HAL_PERIPH_ABP2_SENSORS, averages each sensor's reading, prints every sensor in
  Pascals, and broadcasts the primary sensor over DroneCAN. See abp2_pressure.h
  for the configuration macros and abp2_pressure_backend.* for the wire protocol.
*/

#include "AP_Periph.h"

#if AP_PERIPH_ABP2_PRESSURE_ENABLED

#include "abp2_pressure.h"
#include "abp2_pressure_backend.h"
#include <dronecan_msgs.h>

#ifndef HAL_PERIPH_ABP2_SENSORS
#error "AP_PERIPH_ABP2_PRESSURE_ENABLED requires HAL_PERIPH_ABP2_SENSORS to be defined in hwdef.dat"
#endif

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

#ifndef ABP2_BCAST_MS
#define ABP2_BCAST_MS 100    // 10 Hz DroneCAN broadcast of the primary sensor
#endif
#ifndef ABP2_PRINT_MS
#define ABP2_PRINT_MS 200   // 1 Hz can_printf of every sensor (in Pascals)
#endif

// the per-board sensor table, supplied by hwdef.dat
static const ABP2Pressure::SensorConfig sensor_table[] = HAL_PERIPH_ABP2_SENSORS;

ABP2Pressure::~ABP2Pressure()
{
    for (uint8_t i = 0; i < num_sensors; i++) {
        delete backends[i];
    }
}

static const char *type_str(uint8_t type)
{
    switch (type) {
    case ABP2_ABSOLUTE:     return "abs";
    case ABP2_DIFFERENTIAL: return "diff";
    case ABP2_GAUGE:
    default:                return "gauge";
    }
}

// build the backends from the hwdef table
void ABP2Pressure::init(void)
{
    uint8_t count = ARRAY_SIZE(sensor_table);
    if (count > ABP2_MAX_SENSORS) {
        count = ABP2_MAX_SENSORS;
    }

    for (uint8_t i = 0; i < count; i++) {
        const SensorConfig &cfg = sensor_table[i];
        ABP2_Backend *backend = nullptr;

        switch (cfg.bus_type) {
        case ABP2_BUS_I2C:
            backend = NEW_NOTHROW ABP2_I2C(*this, i, cfg);
            break;
        case ABP2_BUS_SPI:
            backend = NEW_NOTHROW ABP2_SPI(*this, i, cfg);
            break;
        default:
            can_printf("ABP2[%u]: unknown bus_type %u", i, cfg.bus_type);
            break;
        }

        if (backend == nullptr) {
            continue;
        }
        if (!backend->init()) {
            delete backend;
            continue;
        }
        backends[num_sensors++] = backend;
    }
}

void ABP2Pressure::update(void)
{
    if (!initialised) {
        initialised = true;
        init();
    }
    if (num_sensors == 0) {
        return;
    }

    // refresh the averaged reading for each sensor
    for (uint8_t i = 0; i < num_sensors; i++) {
        backends[i]->update();
    }

    const uint32_t now = AP_HAL::millis();

    // broadcast the primary (first) sensor over DroneCAN. The standard
    // StaticPressure/StaticTemperature messages carry no instance field, so only
    // the primary sensor is broadcast; all sensors are reported via can_printf.
    if (now - last_bcast_ms >= ABP2_BCAST_MS) {
        last_bcast_ms = now;
        ABP2_Backend &primary = *backends[0];
        if (primary.healthy()) {
            {
                uavcan_equipment_air_data_StaticPressure pkt {};
                pkt.static_pressure          = primary.pressure();
                pkt.static_pressure_variance = 0;
                uint8_t buf[UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_MAX_SIZE];
                uint16_t len = uavcan_equipment_air_data_StaticPressure_encode(&pkt, buf, !periph.canfdout());
                periph.canard_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_SIGNATURE,
                                        UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_ID,
                                        CANARD_TRANSFER_PRIORITY_LOW,
                                        buf, len);
            }
            {
                uavcan_equipment_air_data_StaticTemperature pkt {};
                pkt.static_temperature          = C_TO_KELVIN(primary.temperature());
                pkt.static_temperature_variance = 0;
                uint8_t buf[UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_MAX_SIZE];
                uint16_t len = uavcan_equipment_air_data_StaticTemperature_encode(&pkt, buf, !periph.canfdout());
                periph.canard_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_SIGNATURE,
                                        UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_ID,
                                        CANARD_TRANSFER_PRIORITY_LOW,
                                        buf, len);
            }
        }
    }

    // print every sensor's reading in Pascals, over both the CAN debug channel
    // and the USB console (hal.console -> OTG1 CDC, same as bldc_motor.cpp)
    if (now - last_print_ms >= ABP2_PRINT_MS) {
        last_print_ms = now;
        for (uint8_t i = 0; i < num_sensors; i++) {
            ABP2_Backend &b = *backends[i];
            if (!b.healthy()) {
                can_printf("ABP2[%u]: no data", i);
                hal.console->printf("ABP2[%u]: no data\n", i);
                continue;
            }
            can_printf("ABP2[%u] %s P=%.1f Pa T=%.2f C",
                       i, type_str(b.config().type),
                       (double)b.pressure(), (double)b.temperature());
            hal.console->printf("ABP2[%u] %s P=%.1f Pa T=%.2f C\n",
                                i, type_str(b.config().type),
                                (double)b.pressure(), (double)b.temperature());
        }
    }
}

#endif // AP_PERIPH_ABP2_PRESSURE_ENABLED
