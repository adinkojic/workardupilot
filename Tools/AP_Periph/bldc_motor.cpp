#include "AP_Periph.h"


#if AP_PERIPH_SINGLE_PHASE_BLDC
#include "bldc_motor.h"

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;


SinglePhaseBLDC::SinglePhaseBLDC()
{
    //init code
}

void SinglePhaseBLDC::update()
{
    static uint32_t loop_counter = 0;

    loop_counter++;

    if(loop_counter % 2000 > 1000)//toggle each second
    {
        hal.gpio->write(GPIO_TEST_LED, 0);
    }
    else
    {
        hal.gpio->write(GPIO_TEST_LED, 1);
    }
}

#endif