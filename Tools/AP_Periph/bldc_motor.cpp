#include "AP_Periph.h"


#if AP_PERIPH_SINGLE_PHASE_BLDC
#include "bldc_motor.h"

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

SinglePhaseBLDC::SinglePhaseBLDC()
{
    //constructor
}

void SinglePhaseBLDC::update()
{
    //called every loop
}

#endif