/*
    DRV8243HQRXYRQ1 driver for a single phase brushless DC motor, HW mode
*/

#include "AP_Periph.h"
#include "hal.h"

#if AP_PERIPH_SINGLE_PHASE_BLDC
#include "bldc_motor.h"
#include <hal.h>

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

// EN PWM duty cycle: 0-200 (0% - 100%), 200 steps per 1kHz cycle at 200kHz ISR rate
static volatile uint8_t motor_en_duty = 50;  // 50% default

static void motor_phase_isr(GPTDriver *gptp) {
    static uint32_t ph_counter = 0;
    static uint32_t en_counter = 0;

    // Phase toggle
    if(ph_counter == 0)
    {
        palToggleLine(HAL_GPIO_LINE_GPIO25);  // PB12 = MOTOR_PH
        ph_counter = 6;
    }
    ph_counter--;

    // Software PWM for EN pin at 1kHz (200 ticks @ 200kHz)
    en_counter++;
    if(en_counter >= 100) en_counter = 0;
    palWriteLine(HAL_GPIO_LINE_GPIO26, en_counter < motor_en_duty ? PAL_HIGH : PAL_LOW);
}

static const GPTConfig gpt4_cfg = {
    .frequency = 1000000,       // 1 MHz timer clock (prescaler = 160-1)
    .callback  = motor_phase_isr,
    .cr2       = 0,
    .dier      = 0,
};


void SinglePhaseBLDC::init()
{
    hal.gpio->write(GPIO_MOTOR_DIAG, 0);
    hal.gpio->write(GPIO_MOTOR_SR, 1);
    hal.gpio->write(GPIO_MOTOR_ITRIP, 0);
    hal.gpio->write(GPIO_MOTOR_MODE, 0);

    hal.gpio->write(GPIO_NSLEEP, 1);

    gptStart(&GPTD4, &gpt4_cfg);
    gptStartContinuous(&GPTD4, 5);  // 1MHz / 10 = 100 kHz
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
    
    motor_en_duty = (loop_counter%10000 )*0.01;

    if(now - last_printed > 1000)
    {
        can_printf("SPMD SPI status, M_fault: %d, Time: %d", (int) motor_faulted, (int) now);
        can_printf("Button status: %d", (int) button_pressed);
        last_printed = now;
    }

    blink_led(now);
}

#endif