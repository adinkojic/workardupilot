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

// MOTOR_B_PHASE = PB0 (hall effect sensor)
#define MOTOR_B_PHASE_LINE HAL_GPIO_LINE_GPIO17

// Motor states
enum MotorState : uint8_t {
    MOTOR_STOPPED,
    MOTOR_STARTUP,   // pre-position rotor to a known pole before commutating
    MOTOR_RUNNING,
};

// 50 ms pre-position kick at 200 kHz ISR rate
#define STARTUP_TICKS 10000U

// Shared state between ISR and main thread
static volatile uint8_t    motor_en_duty     = 0;      // 0–100 (%)
static volatile MotorState motor_state       = MOTOR_STOPPED;
static volatile uint32_t   startup_remaining = 0;      // ticks left in startup phase
static volatile uint32_t   hall_edge_count   = 0;      // total transitions, wraps
static volatile uint32_t   half_period_ticks = 0;      // ISR ticks between last two hall edges

static void motor_phase_isr(GPTDriver *gptp) {
    static uint32_t en_counter       = 0;
    static uint32_t ticks_since_edge = 0;
    static bool     last_hall        = false;

    bool hall = palReadLine(MOTOR_B_PHASE_LINE);

    // --- Commutation state machine ---
    switch (motor_state) {
    case MOTOR_STOPPED:
        palWriteLine(HAL_GPIO_LINE_GPIO25, PAL_LOW);
        break;
    case MOTOR_STARTUP:
        // Force PH HIGH to snap rotor to a known position before commutating
        palWriteLine(HAL_GPIO_LINE_GPIO25, PAL_HIGH);
        if (--startup_remaining == 0) {
            motor_state = MOTOR_RUNNING;
        }
        break;
    case MOTOR_RUNNING:
        palWriteLine(HAL_GPIO_LINE_GPIO25, hall ? PAL_HIGH : PAL_LOW);  // MOTOR_PH
        break;
    }

    // --- RPM measurement: time hall edges in ISR ticks ---
    ticks_since_edge++;
    if (hall != last_hall) {
        half_period_ticks = ticks_since_edge;
        ticks_since_edge  = 0;
        hall_edge_count++;
        last_hall = hall;
    }

    // --- EN software PWM at ~2 kHz (100 ticks @ 200 kHz) ---
    en_counter++;
    if (en_counter >= 100) en_counter = 0;
    bool en_on = (motor_state != MOTOR_STOPPED) && (en_counter < (uint32_t)motor_en_duty);
    palWriteLine(HAL_GPIO_LINE_GPIO26, en_on ? PAL_HIGH : PAL_LOW);    // MOTOR_EN
}

static const GPTConfig gpt4_cfg = {
    .frequency = 1000000,       // 1 MHz timer clock → ISR at 200 kHz (period = 5)
    .callback  = motor_phase_isr,
    .cr2       = 0,
    .dier      = 0,
};


void SinglePhaseBLDC::init()
{
    hal.gpio->write(GPIO_MOTOR_DIAG,  0);
    hal.gpio->write(GPIO_MOTOR_SR,    1);
    hal.gpio->write(GPIO_MOTOR_ITRIP, 0);
    hal.gpio->write(GPIO_MOTOR_MODE,  0);
    hal.gpio->write(GPIO_NSLEEP,      1);

    gptStart(&GPTD4, &gpt4_cfg);
    gptStartContinuous(&GPTD4, 5);  // 1 MHz / 5 = 200 kHz ISR
}

// throttle: 0.0 (stop) … 1.0 (full speed)
void SinglePhaseBLDC::set_throttle(float throttle)
{
    throttle = constrain_float(throttle, 0.0f, 1.0f);
    motor_en_duty = (uint8_t)(throttle * 100.0f);

    if (throttle > 0.0f) {
        // Only trigger startup sequence if currently stopped
        if (motor_state == MOTOR_STOPPED) {
            startup_remaining = STARTUP_TICKS;
            motor_state = MOTOR_STARTUP;
        }
    } else {
        motor_state = MOTOR_STOPPED;
    }
}

// Returns RPM estimate based on hall edge timing.
// pole_pairs must be set correctly (default 1).
// Returns 0 if the motor is stopped or no edges have been seen.
uint32_t SinglePhaseBLDC::get_rpm() const
{
    if (motor_state != MOTOR_RUNNING) return 0;
    uint32_t hp = half_period_ticks;    // snapshot volatile once
    if (hp == 0) return 0;
    // Each ISR tick = 5 µs (200 kHz).
    // One revolution = 2 * pole_pairs hall edges → 2 * pole_pairs half-periods.
    // RPM = 60,000,000 / (hp * 5µs * 2 * pole_pairs) = 6,000,000 / (hp * pole_pairs)
    return 6000000UL / (hp * pole_pairs);
}

void SinglePhaseBLDC::blink_led(uint32_t now)
{
    uint16_t cycle_time = 1000;
    uint16_t on_time = 500;
    switch (motor_state) {
        case MOTOR_STOPPED:
            //default
            break;
        case MOTOR_STARTUP:
            cycle_time = 200;
            on_time = 20;
            break;
        case MOTOR_RUNNING:
            cycle_time = 1000;
            on_time = 900;
            break;
    }
    hal.gpio->write(GPIO_TEST_LED, (now % cycle_time < on_time) ? 1 : 0);
}

void SinglePhaseBLDC::clear_fault()
{
    hal.gpio->write(GPIO_NSLEEP, 0);
    uint32_t start = AP_HAL::micros();
    while (AP_HAL::micros() - start < 5) {}
    hal.gpio->write(GPIO_NSLEEP, 1);
}

void SinglePhaseBLDC::update()
{
    static uint32_t last_clr_fault = 0;
    static uint32_t last_printed   = 0;
    static bool     inited         = false;

    uint32_t now = AP_HAL::millis();

    if (!inited) {
        init();
        inited = true;
    }

    bool button_pressed = (hal.gpio->read(GPIO_DEADMAN_BUTTON) == 0);
    bool motor_faulted  = (hal.gpio->read(GPIO_MOTOR_FAULT)    == 0);

    if (motor_faulted && (now - last_clr_fault > 100)) {
        clear_fault();
        last_clr_fault = now;
    }

    hal.gpio->write(GPIO_DRVOFF, !button_pressed);

    set_throttle(button_pressed ? 1.0f : 0.0f);

    if (now - last_printed > 1000) {
        can_printf("RPM: %u  fault: %d  throttle: %u%%  hall_edges: %u",
                   (unsigned)get_rpm(),
                   (int)motor_faulted,
                   (unsigned)motor_en_duty,
                   (unsigned)hall_edge_count);
        last_printed = now;
    }

    blink_led(now);
}

#endif
