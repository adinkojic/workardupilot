/*
    DRV8243HQRXYRQ1 driver for a single phase brushless DC motor, HW mode
*/

#include "AP_Periph.h"
#include "hal.h"

#if AP_PERIPH_SINGLE_PHASE_BLDC
#include "bldc_motor.h"

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

/*
static void timer_callback(GPTDriver *gptp) {
    // Called at 100 kHz from interrupt context
    // Keep this extremely short — no allocation, no blocking
    volatile static uint32_t counter = 0; //in 10s of ms
    static uint32_t start_time = 0;
    const uint32_t clear_time = 10; //10s of us
    static bool clear_underway = false;

    if(!clear_underway){
        start_time = now;
        clear_underway = true;
    }

    if(now - start_time < clear_time && clear_underway)
    {
        hal.gpio->write(GPIO_NSLEEP, 0);
    }
    else
    {
        hal.gpio->write(GPIO_NSLEEP, 1);
        clear_underway = false;
        return true;
    }

    counter++;
}

static const GPTConfig gpt_cfg = {
    .frequency = 1000000,   // 1 MHz timer clock
    .callback  = timer_callback,
    .cr2       = 0,
    .dier      = 0,
};

void start_100khz_timer() {
    gptStart(&GPTD4, &gpt_cfg);          // use an available GPT driver
    gptStartContinuous(&GPTD4, 10);      // 1 MHz / 10 = 100 kHz
}*/

void SinglePhaseBLDC::init()
{
    hal.gpio->write(GPIO_MOTOR_DIAG, 0);
    hal.gpio->write(GPIO_MOTOR_SR, 1);
    hal.gpio->write(GPIO_MOTOR_ITRIP, 0);
    hal.gpio->write(GPIO_MOTOR_MODE, 0);

    hal.gpio->write(GPIO_NSLEEP, 1);
}

void SinglePhaseBLDC::blink_led(uint32_t now)
{
    if(now % 1000 < 100)//toggle each second
    {
        hal.gpio->write(GPIO_TEST_LED, 1);
    }
    else
    {
        hal.gpio->write(GPIO_TEST_LED, 0);
    }
}

//Attempts to clear motor fault
void SinglePhaseBLDC::clear_fault()
{
    hal.gpio->write(GPIO_NSLEEP, 0);
    uint32_t start = AP_HAL::micros();
    while(AP_HAL::micros() - start < 10){
        //spinloop
    }
    hal.gpio->write(GPIO_NSLEEP, 1);

}

void SinglePhaseBLDC::update()
{
    static uint32_t loop_counter = 0;
    static uint32_t last_clr_fault = 0;
    uint32_t now = AP_HAL::millis();
    static uint32_t last_printed = 0;
    static bool inited = false;
    if(!inited)
    {
        init();
        inited = true;
    }

    loop_counter++;
    bool button_pressed = (hal.gpio->read(GPIO_DEADMAN_BUTTON) == 0);
    bool motor_faulted = (hal.gpio->read(GPIO_MOTOR_FAULT) == 0);

    if(motor_faulted && (now - last_clr_fault > 100))
    {
        clear_fault();
        last_clr_fault = now;
    }
    
    hal.gpio->write(GPIO_DRVOFF, !button_pressed);
    hal.gpio->write(GPIO_MOTOR_EN, 1);
    hal.gpio->write(GPIO_MOTOR_PH, 1);

    if(now - last_printed > 1000)
    {
        can_printf("SPMD SPI status, M_fault: %d, Time: %d", (int) motor_faulted, (int) now);
        can_printf("Button status: %d", (int) button_pressed);
        last_printed = now;
    }

    blink_led(now);
}

#endif