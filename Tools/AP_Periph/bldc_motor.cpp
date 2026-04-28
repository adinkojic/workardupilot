/*
    Single-phase BLDC H-bridge driver using TIM8 complementary PWM + predriver.

    All four MCU outputs are active-high; the predriver converts them to the
    correct gate voltages for the H-bridge MOSFETs.

    Pin mapping (TIM8, 160 MHz APB2 clock):
      PC6  = TIM8_CH1  = AL  (Leg A low-side,  CC1P=0: active-high → predriver)
      PC10 = TIM8_CH1N = AH  (Leg A high-side, CC1NP=0: active-high → predriver)
      PB9  = TIM8_CH3  = BL  (Leg B low-side,  CC3P=0: active-high → predriver)
      PB1  = TIM8_CH3N = BH  (Leg B high-side, CC3NP=0: active-high → predriver)

    CH1 uses PWM mode 1 (OCref=1 when CNT < CCR), CH3 uses PWM mode 2 (OCref=1
    when CNT >= CCR).  Both legs therefore assert their active switch in the same
    region of the centre-aligned triangle wave, giving simultaneous conduction:

      Forward (dir=+1): CCR1=CCR3 = ARR−duty
        → AH+BL both HIGH when CNT >= ARR−duty  (active, duty/ARR fraction)
        → AL+BH both HIGH when CNT <  ARR−duty  (synchronous freewheeling)

      Reverse (dir=−1): CCR1=CCR3 = duty
        → AL+BH both HIGH when CNT <  duty      (active, duty/ARR fraction)
        → AH+BL both HIGH when CNT >= duty      (synchronous freewheeling)

    Dead time on each half-bridge is enforced independently by TIM8 BDTR.

    Dead time: DEAD_TIME_TICKS inserted by TIM8 BDTR (MCU-side).
               Total system dead time = MCU dead time + predriver propagation delay.
    Bridge off: MOE=0, all outputs held LOW.

    Operating modes:
      STARTUP – 50 ms kick pulse, then hall-commuted open-loop spin-up until
                STARTUP_RPM_THRESHOLD.
      PID     – hall commutation continues in the 100 kHz ISR; main loop runs
                PID to set duty cycle.

    Hall sensor (PC4 / GPIO_MOTOR_A_PHASE) is polled by a 100 kHz TIM4 GPT
    callback.  Each edge triggers an immediate H-bridge direction update and
    records the half-period for RPM measurement.  The 1 kHz main loop only
    needs to compute RPM and run the PID — no commutation timing constraint.
*/

#include "AP_Periph.h"
#include "hal.h"

#if AP_PERIPH_SINGLE_PHASE_BLDC
#include "bldc_motor.h"

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

#define TIM8_CLK_HZ     160000000UL
#define DEFAULT_FREQ_HZ 20000UL
#define DEAD_TIME_TICKS 80U          // 500 ns @ 160 MHz

// Hall sensor line — resolved from hwdef (StratospheresBBLE2: PB15)
#define HALL_LINE HAL_GPIO_PIN_MOTOR_A_PHASE

// Bridge idle: dead-time + off-state drive, no MOE
static const uint32_t BDTR_BASE = TIM_BDTR_OSSR | TIM_BDTR_OSSI | DEAD_TIME_TICKS;

// ---------------------------------------------------------------------------
// ISR-shared state  (written/read from the 100 kHz TIM4 callback)
// ---------------------------------------------------------------------------

// ISR fires at 100 kHz → 10 µs per tick.
// At 500 000 electrical RPM: half-period ≈ 60 µs → ~6 ticks — well resolved.
#define ISR_TICK_US        10U
#define RPM_TIMEOUT_TICKS  (RPM_TIMEOUT_US / ISR_TICK_US)   // 50 000 ticks

static volatile uint32_t s_tick              = 0;    // monotonic ISR tick counter
static volatile uint32_t s_half_period_ticks = 0;    // ticks between last two edges
static volatile uint32_t s_last_edge_tick    = 0;    // tick of most recent edge
static volatile bool     s_last_hall         = false;
static volatile bool     s_commutate_en      = false; // ISR commutation active
static volatile int8_t   s_direction         = 1;    // current H-bridge polarity
static volatile float    s_throttle_v        = 0.0f; // mirrored for ISR
static volatile uint32_t s_arr_v             = 4000U; // mirrored for ISR

// 100 kHz TIM4 ISR — direct register access, no GPT driver needed.
// APB1 = 160 MHz (PPRE1=DIV1): PSC=159 → 1 MHz counter, ARR=9 → 10 µs period.
CH_IRQ_HANDLER(STM32_TIM4_HANDLER)
{
    CH_IRQ_PROLOGUE();
    TIM4->SR = 0;   // clear update flag

    s_tick++;

    bool hall = (palReadLine(HALL_LINE) != 0);

    if (hall != s_last_hall) {
        uint32_t dt = s_tick - s_last_edge_tick;
        if (dt > 0) {
            s_half_period_ticks = dt;
        }
        s_last_edge_tick = s_tick;
        s_last_hall      = hall;

        if (s_commutate_en) {
            s_direction   = hall ? 1 : -1;
            uint32_t arr  = s_arr_v;
            uint32_t duty = (uint32_t)(s_throttle_v * arr);
            uint32_t ccr  = (s_direction > 0) ? (arr - duty) : duty;
            TIM8->CCR1    = ccr;
            TIM8->CCR3    = ccr;
        }
    }

    CH_IRQ_EPILOGUE();
}

static void tim4_init()
{
    rccEnableTIM4(true);
    TIM4->CR1  = 0;
    TIM4->PSC  = 159U;           // 160 MHz / 160 = 1 MHz counter clock
    TIM4->ARR  = 9U;             // 1 MHz / 10 = 100 kHz update event
    TIM4->DIER = TIM_DIER_UIE;   // enable update interrupt
    TIM4->EGR  = TIM_EGR_UG;    // load PSC/ARR immediately
    TIM4->SR   = 0;
    TIM4->CR1  = TIM_CR1_CEN;
    nvicEnableVector(TIM4_IRQn, STM32_IRQ_TIM4_PRIORITY);
}

// ---------------------------------------------------------------------------
// TIM8 helpers
// ---------------------------------------------------------------------------

// Writes TIM8 CCRs from current _throttle / s_direction and syncs volatile mirrors.
void SinglePhaseBLDC::ccr_apply()
{
    s_throttle_v  = _throttle;
    s_arr_v       = _arr;
    uint32_t duty = (uint32_t)(_throttle * _arr);
    // Forward: CCR = ARR-duty → AH+BL active when CNT >= CCR (top of triangle)
    // Reverse: CCR = duty    → AL+BH active when CNT <  CCR (bottom of triangle)
    uint32_t ccr  = (s_direction > 0) ? (_arr - duty) : duty;
    TIM8->CCR1    = ccr;
    TIM8->CCR3    = ccr;
}

static void tim8_init(uint32_t arr)
{
    rccEnableTIM8(true);

    TIM8->CR1  = 0;
    TIM8->CR2  = 0;   // OIS = 0: idle state LOW for all channels
    TIM8->SMCR = 0;

    // Center-aligned mode 1, auto-reload preload enabled
    TIM8->CR1 = TIM_CR1_CMS_0 | TIM_CR1_ARPE;

    TIM8->PSC = 0;
    TIM8->ARR = arr - 1;
    TIM8->RCR = 0;

    // CH1: PWM mode 1 (OCref=1 when CNT < CCR) — AH active in top region
    // CH3: PWM mode 2 (OCref=1 when CNT >= CCR) — BL active in top region
    // Both channels share the same CCR value so AH+BL (or AL+BH) overlap exactly.
    TIM8->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM8->CCMR2 = (7U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    TIM8->CCR1 = 0;
    TIM8->CCR3 = 0;

    // All outputs active-high; predriver handles gate voltage conversion.
    // CC1P=0: CH1 (AL) active-high; CC1NP=0: CH1N (AH) active-high complement.
    // CC3P=0: CH3 (BL) active-high; CC3NP=0: CH3N (BH) active-high complement.
    TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC3E | TIM_CCER_CC3NE;

    TIM8->BDTR = BDTR_BASE;   // bridge off
    TIM8->EGR  = TIM_EGR_UG;
    TIM8->SR   = 0;
    TIM8->CR1 |= TIM_CR1_CEN;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void SinglePhaseBLDC::init()
{
    _arr   = TIM8_CLK_HZ / (2U * DEFAULT_FREQ_HZ);
    s_arr_v = _arr;
    tim8_init(_arr);
    ccr_apply();

    tim4_init();

    set_test_mode(true);
}

void SinglePhaseBLDC::set_enabled(bool en)
{
    _enabled   = en;
    TIM8->BDTR = en ? (BDTR_BASE | TIM_BDTR_MOE) : BDTR_BASE;
}

void SinglePhaseBLDC::set_throttle(float throttle)
{
    _throttle = constrain_float(throttle, 0.0f, 1.0f);
    ccr_apply();
}

void SinglePhaseBLDC::set_frequency(uint32_t hz)
{
    _arr      = TIM8_CLK_HZ / (2U * hz);
    TIM8->ARR = _arr - 1;
    ccr_apply();
    TIM8->EGR = TIM_EGR_UG;
    TIM8->SR  = 0;
}

void SinglePhaseBLDC::flip_direction()
{
    // Disable bridge, swap phase, reset counter — also updates s_direction
    TIM8->BDTR = BDTR_BASE;
    s_direction = -s_direction;
    ccr_apply();
    TIM8->EGR  = TIM_EGR_UG;
    TIM8->SR   = 0;
}

// ---------------------------------------------------------------------------
// RPM (called from main loop — reads ISR volatile state)
// ---------------------------------------------------------------------------

void SinglePhaseBLDC::compute_rpm()
{
    // Snapshot volatile ISR state; minor race is acceptable (one-tick error max)
    uint32_t half_ticks  = s_half_period_ticks;
    uint32_t age_ticks   = s_tick - s_last_edge_tick;

    if (half_ticks == 0 || age_ticks >= RPM_TIMEOUT_TICKS) {
        _actual_rpm = 0.0f;
        return;
    }
    // Two edges per electrical cycle
    float period_s = 2.0f * half_ticks * ISR_TICK_US * 1e-6f;
    _actual_rpm = 60.0f / (period_s * POLE_PAIRS);
}

// ---------------------------------------------------------------------------
// Mode state machines (called from main loop at ~1 kHz)
// ---------------------------------------------------------------------------

void SinglePhaseBLDC::update_startup()
{
    if (!_kick_done) {
        // Phase 1: fixed-direction kick to break static friction
        if (_kick_start_ms == 0) {
            _kick_start_ms = AP_HAL::millis();
        }
        s_commutate_en = false;
        set_throttle(STARTUP_KICK_THROTTLE);
        set_enabled(true);
        if (AP_HAL::millis() - _kick_start_ms >= STARTUP_KICK_MS) {
            _kick_done = true;
        }
        return;
    }

    // Phase 2: ISR handles commutation; main loop just holds throttle
    s_commutate_en = true;
    set_throttle(STARTUP_COMMUTE_THROTTLE);
    set_enabled(true);

    if (_actual_rpm >= STARTUP_RPM_THRESHOLD) {
        _mode          = DriveMode::PID;
        _pid_integral  = 0.0f;
        _pid_last_err  = TARGET_RPM - _actual_rpm;
        _pid_last_us   = AP_HAL::micros();
    }
}

void SinglePhaseBLDC::update_pid()
{
    // ISR continues handling commutation; PID controls duty cycle only
    s_commutate_en = true;
    set_enabled(true);

    uint32_t now_us = AP_HAL::micros();
    float dt = (now_us - _pid_last_us) * 1e-6f;
    _pid_last_us = now_us;

    if (dt <= 0.0f || dt > 0.5f) {
        return;
    }

    float err      = TARGET_RPM - _actual_rpm;
    _pid_integral += err * dt;
    _pid_integral  = constrain_float(_pid_integral,
                                     -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    float deriv    = (err - _pid_last_err) / dt;
    _pid_last_err  = err;

    float duty = PID_KP * err + PID_KI * _pid_integral + PID_KD * deriv;
    set_throttle(constrain_float(duty, 0.0f, 1.0f));
}

void SinglePhaseBLDC::update_stopping()
{
    // Keep commutating while the rotor is still spinning
    s_commutate_en = true;
    set_enabled(true);

    float elapsed_s = (AP_HAL::millis() - _stop_start_ms) * 1e-3f;
    float ramp_s    = STOP_RAMP_MS * 1e-3f;
    float duty      = _stop_start_throttle * (1.0f - elapsed_s / ramp_s);

    if (duty <= 0.0f) {
        set_throttle(0.0f);
        set_enabled(false);
        s_commutate_en = false;
        // Reset so the next button press starts fresh
        _mode          = DriveMode::IDLE;
        _kick_done     = false;
        _kick_start_ms = 0;
        _pid_integral  = 0.0f;
    } else {
        set_throttle(duty);
    }
}

// ---------------------------------------------------------------------------
// Test mode (called from main loop at ~1 kHz)
// Bridge follows the deadman button directly; each press flips direction.
// No hall commutation or PID — purely for H-bridge verification.
// ---------------------------------------------------------------------------

void SinglePhaseBLDC::update_test()
{
    bool deadman = (hal.gpio->read(GPIO_DEADMAN_BUTTON) == 0);

    // Rising edge: flip direction before enabling so the first energisation
    // goes in the new direction.
    if (deadman && !_prev_deadman) {
        s_direction = -s_direction;
        s_commutate_en = false;
        set_throttle(TEST_THROTTLE);
        ccr_apply();
    }
    _prev_deadman = deadman;

    if (deadman) {
        set_enabled(true);
    } else {
        set_enabled(false);
        set_throttle(0.0f);
        _mode = DriveMode::IDLE;
    }
}

// ---------------------------------------------------------------------------
// Main update (called from AP_Periph loop at ~1 kHz)
// ---------------------------------------------------------------------------

void SinglePhaseBLDC::update()
{
    static bool inited = false;
    if (!inited) {
        init();
        inited = true;
    }

    // Low = pressed
    bool deadman = (hal.gpio->read(GPIO_DEADMAN_BUTTON) == 0);

    compute_rpm();

    // Deadman released mid-run → begin controlled ramp-down (normal modes only)
    if (!deadman && (_mode == DriveMode::STARTUP || _mode == DriveMode::PID)) {
        _mode                = DriveMode::STOPPING;
        _stop_start_ms       = AP_HAL::millis();
        _stop_start_throttle = _throttle;
    }

    switch (_mode) {
    case DriveMode::IDLE:
        // Wait for button press; test mode bypasses the normal startup sequence
        if (deadman) {
            _mode = _test_mode_active ? DriveMode::TEST : DriveMode::STARTUP;
        }
        break;
    case DriveMode::STARTUP:  update_startup();  break;
    case DriveMode::PID:      update_pid();      break;
    case DriveMode::STOPPING: update_stopping(); break;
    case DriveMode::TEST:     update_test();     break;
    }

    blink_led(_mode == DriveMode::PID);

#if AP_PERIPH_BATTERY_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _batt_print_ms >= 200) {
        _batt_print_ms = now_ms;
        if (periph.battery_lib.healthy(0)) {
            float current;
            if (periph.battery_lib.current_amps(current, 0)) {
                hal.console->printf("BATT: %.2fV %.3fA\n",
                                    (double)periph.battery_lib.voltage(0),
                                    (double)current);
            } else {
                hal.console->printf("BATT: %.2fV\n",
                                    (double)periph.battery_lib.voltage(0));
            }
        }
    }
#endif

    hal.console->printf("HALL: %.1fV\n", (double) s_last_hall);
}

void SinglePhaseBLDC::blink_led(bool on)
{
    hal.gpio->write(GPIO_TEST_LED, on);
}

#endif
