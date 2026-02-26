/*
    DRV8243SQRXYRQ1 driver for a single phase brushless DC motor, SPI mode
*/

#include "AP_Periph.h"

#if AP_PERIPH_SINGLE_PHASE_BLDC
#include "bldc_motor.h"

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;


void SinglePhaseBLDC::init()
{
    _spi = hal.spi->get_device("motor_driver");
}

//send clear fault command over SPI
bool SinglePhaseBLDC::clear_fault() 
{
    uint8_t clr_flt_command[2] = {0b0001000, 0b10001001};
    uint8_t dummy[2];

    if (!_spi->get_semaphore()->take(0)) {
        return false;
    }

    bool result = _spi->transfer(clr_flt_command, sizeof(clr_flt_command), dummy, sizeof(dummy));

    _spi->get_semaphore()->give();

    return result;
}

void SinglePhaseBLDC::update()
{
    static uint32_t loop_counter = 0;
    static uint32_t last_clr_fault_attempt = 0;
    uint32_t now = AP_HAL::millis();
    static bool inited = false;
    if(!inited)
    {
        init();
        inited = true;
    }

    loop_counter++;
    bool button_pressed = (hal.gpio->read(GPIO_DEADMAN_BUTTON) == 0);
    bool motor_faulted = (hal.gpio->read(GPIO_MOTOR_FAULT) == 0);

    if(motor_faulted && (now - last_clr_fault_attempt > 100))
    {
        last_clr_fault_attempt = now;
        clear_fault();
        
    }

    hal.gpio->write(GPIO_NSLEEP, button_pressed);
    hal.gpio->write(GPIO_DRVOFF, 1);
    //hal.gpio->write(GPIO_MOTOR_EN, 1);

    if(loop_counter % 1000 > 100)//toggle each second
    {
        hal.gpio->write(GPIO_MOTOR_EN, 0);
        hal.gpio->write(GPIO_TEST_LED, motor_faulted);
    }
    else
    {
        hal.gpio->write(GPIO_MOTOR_EN, 1);
        hal.gpio->write(GPIO_TEST_LED, !motor_faulted);
    }
}

#endif