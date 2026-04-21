#include "AP_Periph.h"

#if AP_PERIPH_BARO_ENABLED

/*
  barometer support
 */

#include <dronecan_msgs.h>

/*
  update CAN baro
 */
void AP_Periph_FW::can_baro_update(void)
{
    if (!periph.g.baro_enable) {
        return;
    }
    baro.update();
    if (last_baro_update_ms == baro.get_last_update()) {
        return;
    }

    last_baro_update_ms = baro.get_last_update();
    if (!baro.healthy()) {
        // don't send any data
        return;
    }
    const float press = baro.get_pressure();
    const float temp = baro.get_temperature();

    static uint32_t last_baro_print_ms;
    const uint32_t now = AP_HAL::millis();
    if (now - last_baro_print_ms >= 1000) {
        last_baro_print_ms = now;
        can_printf("MPRLS: P=%.0f Pa (%.2f PSI) T=%.1f C",
                   (double)press,
                   (double)(press / 6894.757f),
                   (double)temp);
    }

    {
        uavcan_equipment_air_data_StaticPressure pkt {};
        pkt.static_pressure = press;
        pkt.static_pressure_variance = 0; // should we make this a parameter?

        uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_air_data_StaticPressure_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }

    {
        uavcan_equipment_air_data_StaticTemperature pkt {};
        pkt.static_temperature = C_TO_KELVIN(temp);
        pkt.static_temperature_variance = 0; // should we make this a parameter?

        uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_air_data_StaticTemperature_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
}

#endif // AP_PERIPH_BARO_ENABLED
