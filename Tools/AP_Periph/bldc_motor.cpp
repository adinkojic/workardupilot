#include "AP_Periph.h"


#if AP_PERIPH_SINGLE_PHASE_BLDC
#include "bldc_motor.h"

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;


SinglePhaseBLDC::SinglePhaseBLDC()
{
    // crashses here all time time hal.gpio->pinMode(GPIO_DEADMAN_BUTTON, HAL_GPIO_INPUT);
}

void SinglePhaseBLDC::update()
{
    static uint32_t loop_counter = 0;

    loop_counter++;
    bool button_pressed = (hal.gpio->read(GPIO_DEADMAN_BUTTON) == 0);

    if(loop_counter % 1000 > 100)//toggle each second
    {
        hal.gpio->write(GPIO_TEST_LED, button_pressed);
    }
    else
    {
        hal.gpio->write(GPIO_TEST_LED, !button_pressed);
    }
}

#endif