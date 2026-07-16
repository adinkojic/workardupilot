/*
    Single-phase BLDC H-bridge driver — unipolar PWM with synchronous freewheel.

    All four MCU outputs are active-high; the predriver converts them to the
    correct gate voltages for the H-bridge MOSFETs.

    Pin mapping (TIM8, 160 MHz APB2 clock):
      PC6  = TIM8_CH1  = AL  (Leg A low-side,  CC1P=0: active-high → predriver)
      PC10 = TIM8_CH1N = AH  (Leg A high-side, CC1NP=0: active-high → predriver)
      PB9  = TIM8_CH3  = BL  (Leg B low-side,  CC3P=0: active-high → predriver)
      PB1  = TIM8_CH3N = BH  (Leg B high-side, CC3NP=0: active-high → predriver)

    PWM scheme: the A leg is PWM'd (CH1 = PWM mode 1, CH1N = complement with
    dead-time). The B leg is held static via OC3M = Force active / Force
    inactive — its output never transitions during a PWM cycle, so the only
    switching losses are on the A leg. Commutation just swaps which side of
    the B leg is the static rail and inverts the CCR1 phasing:

      Forward (s_direction = +1):  OC3M = Force ACTIVE   → BL=1, BH=0 static
                                   CCR1 = ARR − duty
        Top of triangle    (duty fraction):     AH=1, BL=1  → current A→B  (active)
        Bottom of triangle (1−duty fraction):   AL=1, BL=1  → low-side short  (synchronous freewheel)

      Reverse (s_direction = −1):  OC3M = Force INACTIVE → BL=0, BH=1 static
                                   CCR1 = duty
        Bottom of triangle (duty fraction):     AL=1, BH=1  → current B→A  (active)
        Top of triangle    (1−duty fraction):   AH=1, BH=1  → high-side short (synchronous freewheel)

    Throttle = 0 → CCR1 saturates so the A leg sits entirely in the freewheel
    half of the cycle; both terminals tied to the same rail, zero net drive,
    inductor current decays through MOSFET channels (synchronous rectification).
    Throttle = 1 → A leg pinned to the active half; full drive.

    Dead-time: DEAD_TIME_TICKS inserted by TIM8 BDTR on the CH1/CH1N pair.
               OC3REF is held static so the CH3/CH3N pair has no dead-time
               events under normal operation. Total system dead-time =
               MCU dead-time + predriver propagation.
    Bridge off: MOE=0, all outputs held LOW (OIS=0).

    Operating modes:
      STARTUP – brief kick pulse (STARTUP_KICK_MS), then hall-commuted open-loop
                spin-up until STARTUP_RPM_THRESHOLD (electrical RPM). Commutation
                is immediate (right at the hall edge) and the A leg is PWM-chopped
                at STARTUP_COMMUTE_THROTTLE during startup.
      PID     – the current is no longer chopped. Each hall edge schedules a
                single H-bridge pulse (see "Pulse-width drive" below); the main
                loop runs PID on electrical RPM (or follows RC throttle) to set
                the pulse width via the throttle mirror.

    Hall sensor (PB15 / GPIO_MOTOR_PHASE) drives a GPIO edge interrupt (EXTI,
    both edges) via hal.gpio->attach_interrupt(). The handler fires on each hall
    flip and timestamps the edge (µs) so the 1 kHz main loop can derive RPM from
    the inter-edge interval.

    Pulse-width drive (PID mode): once spun up the driver stops chopping. The
    hall handler keeps a rolling average of the half period (one hall pulse
    width) over the last SPEED_AVG_CYCLES edges and, on every edge, arms two
    TIM5 one-shots off the free-running 32-bit counter:
      CC1 = turn ON  at edge + 1/4 pulse width (= 1/8 electrical cycle) + offset
      CC2 = turn OFF at ON + throttle × 1/2 pulse width (≤ PULSE_MAX_ON_US)
    The ON compare drives the H-bridge fully active (no PWM chop) in the
    hall-selected direction and asserts MOE; the OFF compare drops MOE so the
    winding floats for the rest of the cycle. Throttle therefore sweeps the
    drive window across the middle 50% of each hall pulse, with the window
    hard-capped at PULSE_MAX_ON_US since a full-rail pulse into a slow rotor
    is current-limited only by winding resistance. Both edges arm a fresh
    single-shot, so if edges stop the bridge simply stays disabled; a half
    period longer than ~2 electrical cycles is treated as a stall and resets
    the speed estimate, while one shorter than 1/4 of the average is treated
    as hall bounce and ignored entirely (no pulse, average untouched).
    PULSE_DELAY_OFFSET_US adds a fixed lead/lag to the ON instant (0 for now).
    The hall level still selects the drive direction through hall_dir(),
    unchanged from startup.

    Simulation mode (SBUS channel 5 high): the real hall sensor is replaced by
    a simulated rotor while the H-bridge stays live, so the whole drive stack
    above runs unchanged against fake hall data (see the "Simulation mode"
    section and bldc_motor.h for the model). A battery current gulp while
    simulating (motor unexpectedly connected) stops the drive and locks it out
    for 10 s with the LED flashing rapidly.
*/

#include "AP_Periph.h"
#include "hal.h"

#if AP_PERIPH_SINGLE_PHASE_BLDC
#include "bldc_motor.h"

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

#define TIM8_CLK_HZ     160000000UL
#define DEFAULT_FREQ_HZ 20000UL

// TIM5 is a free 32-bit general-purpose timer (APB1, 160 MHz). It runs as a
// free-running up-counter at the full 160 MHz (PSC = 0 → 6.25 ns/tick) and
// provides the two one-shot compares that bracket each hall drive pulse:
//   CC1 → turn the H-bridge ON  (armed on every hall edge)
//   CC2 → turn the H-bridge OFF (armed on every hall edge)
// Both are absolute compares against the free-running counter, so each is
// inherently one-shot: a target is not revisited for ~27 s, so a channel simply
// waits, disarmed in effect, until the hall handler writes a fresh target. The
// handler is suppressed by the ChibiOS HAL (STM32_TIM5_SUPPRESS_ISR), so the
// vector is ours to define.
#define TIM5_CLK_HZ              160000000UL
#define PULSE_DELAY_OFFSET_TICKS ((PULSE_DELAY_OFFSET_US) * (TIM5_CLK_HZ / 1000000UL))
#define PULSE_MAX_ON_TICKS       ((PULSE_MAX_ON_US) * (TIM5_CLK_HZ / 1000000UL))
// DTG field is 8-bit non-linear: DTG[7:5]=111 → DT = (32 + DTG[4:0]) × 16 × t_DTS
// 0xF2 = (32+18)×16×6.25 ns = 5000 ns @ 160 MHz
#define DEAD_TIME_TICKS 0xF2U

// Hall sensor line — resolved from hwdef (StratospheresBBLE2: PB15)
#define HALL_LINE HAL_GPIO_PIN_MOTOR_PHASE

// CCMR2 values selecting the B-leg static rail.
//   FWD: OC3REF forced high → BL=1, BH=0  (freewheel via low-side short)
//   REV: OC3REF forced low  → BL=0, BH=1  (freewheel via high-side short)
// OC3M = 0b0101 (Force active) / 0b0100 (Force inactive). Other CCMR2 bits
// intentionally zero: CC3S=output, no preload — neither matters under force
// mode since CCR3 is unused.
#define CCMR2_FWD_STATIC  (5U << TIM_CCMR2_OC3M_Pos)
#define CCMR2_REV_STATIC  (4U << TIM_CCMR2_OC3M_Pos)

// Bridge idle: dead-time + off-state drive, no MOE
static const uint32_t BDTR_BASE = TIM_BDTR_OSSR | TIM_BDTR_OSSI | DEAD_TIME_TICKS;

// ---------------------------------------------------------------------------
// ISR-shared state  (written/read from the hall-edge EXTI handler)
// ---------------------------------------------------------------------------

static volatile uint32_t s_half_period_us = 0;     // µs between the last two hall edges
static volatile uint32_t s_last_edge_us   = 0;     // micros() timestamp of most recent edge
static volatile bool     s_commutate_en   = false; // immediate (startup) commutation active
static volatile int8_t   s_direction      = 1;     // current H-bridge polarity
static volatile float    s_throttle_v     = 0.0f;  // mirrored for ISR
static volatile uint32_t s_arr_v          = 4000U; // mirrored for ISR

// Pulse-width drive (PID mode). When s_pulse_en is set the hall EXTI handler
// stops commutating inline; instead each edge arms two TIM5 one-shots (CC1 =
// ON, CC2 = OFF) that bracket a single drive pulse. s_last_edge_ticks is the
// TIM5 count at the previous edge; the inter-edge tick count is the half period
// (one hall pulse width) at full timer resolution. s_pulse_primed gates the
// first edge, which only establishes a baseline tick count (no half period to
// schedule from yet). s_pulse_dir latches the hall-selected drive direction at
// the arming edge so the ON compare drives the correct way. The s_half_ticks_*
// set keeps a rolling average of the half period over the last SPEED_AVG_CYCLES
// edges (running sum + ring buffer).
static volatile bool     s_pulse_en        = false;
static volatile bool     s_pulse_primed    = false;
static volatile int8_t   s_pulse_dir       = 1;
static volatile uint32_t s_last_edge_ticks = 0;
static volatile uint32_t s_half_ticks_buf[SPEED_AVG_CYCLES] = {};
static volatile uint32_t s_half_ticks_sum  = 0;
static volatile uint8_t  s_half_ticks_idx  = 0;
static volatile uint8_t  s_half_ticks_cnt  = 0;

// SBUS edge capture (PB13). The EXTI handler measures the width of each
// completed high/low state and queues (high, low) pairs in a single-producer
// single-consumer ring buffer; update_sbus() drains it into AP_RCProtocol.
// Sizing: an SBUS frame is 25 bytes × 12 bits ≈ 3 ms, worst case ~150 pulse
// pairs, repeating every ~9 ms — 256 entries (1 KiB) gives ample headroom
// for the 1 kHz drain rate.
struct sbus_pulse {
    uint16_t high_us;
    uint16_t low_us;
};
#define SBUS_PULSE_BUF_SIZE 256U   // must be a power of two
static sbus_pulse        s_sbus_buf[SBUS_PULSE_BUF_SIZE];
static volatile uint16_t s_sbus_head    = 0;   // ISR write index
static volatile uint16_t s_sbus_tail    = 0;   // main-loop read index
static volatile uint32_t s_sbus_edge_us = 0;   // timestamp of previous edge
static volatile uint32_t s_sbus_high_us = 0;   // width of last completed high state

// Simulation mode (SBUS channel 5 high). The real hall EXTI is ignored and
// hall_irq() is fed simulated edges instead — everything downstream (RPM
// estimation, startup commutation, TIM5 pulse scheduler, PID) runs unchanged
// and the H-bridge stays live. Above SIM_ISR_HANDOFF_RPM the edges are chained
// by the TIM5 CC3 compare ISR: each firing toggles the fake hall, feeds it to
// hall_irq() with SIM_HALL_FAKE_PIN as the pin id, and schedules the next
// edge s_sim_half_ticks later. Below the handoff speed the 1 kHz main loop
// emits edges from the integrated rotor angle instead (s_sim_half_ticks = 0
// lets the ISR chain die; s_sim_isr_armed tells the main loop it has). The
// rotor model itself (_sim_mech_rpm etc.) lives in the class and is integrated
// only from update_simulation().
static volatile bool     s_sim_active     = false; // fake hall replaces the real one
static volatile bool     s_sim_hall       = false; // current simulated hall level
static volatile uint32_t s_sim_half_ticks = 0;     // TIM5 ticks between edges (0 = chain dies)
static volatile bool     s_sim_isr_armed  = false; // CC3 edge chain currently running
static volatile uint32_t s_sim_edge_count = 0;     // simulated edges since sim entry
static SinglePhaseBLDC  *s_sim_owner      = nullptr; // instance for the CC3 ISR

// Set the simulated hall level, mirroring it onto the SIM_HALL debug pin
// (PB12 on boards that define one) so the fake sensor can be scoped alongside
// the H-bridge outputs. Called from both the CC3 ISR and the main loop.
static inline void sim_hall_set(bool level)
{
    s_sim_hall = level;
#ifdef HAL_GPIO_PIN_SIM_HALL
    palWriteLine(HAL_GPIO_PIN_SIM_HALL, level ? PAL_HIGH : PAL_LOW);
#endif
}

// Hall level → drive direction, gated by the single HALL_POLARITY constant.
// This is the one mapping used everywhere commutation direction is derived from
// the hall (startup kick, immediate commutation, phase scheduler), so flipping
// HALL_POLARITY reverses the sensor relationship for all of them at once.
static inline int8_t hall_dir(bool pin_state)
{
    return (int8_t)((pin_state ? 1 : -1) * HALL_POLARITY);
}

// Drive the H-bridge for the given direction at the current mirrored duty.
// This is the startup commutation primitive: it writes the CCR1 phasing and
// the B-leg static rail for `dir`, PWM-chopping the A leg at s_throttle_v.
// Called inline from the hall handler in immediate (startup) mode. Kept in RAM
// so commutation latency is not at the mercy of flash wait states.
__RAMFUNC__ static inline void commutate_now(int8_t dir)
{
    s_direction = dir;
    const uint32_t arr  = s_arr_v;
    const uint32_t duty = (uint32_t)(s_throttle_v * arr);
    if (dir > 0) {
        TIM8->CCR1  = arr - duty;
        TIM8->CCMR2 = CCMR2_FWD_STATIC;
    } else {
        TIM8->CCR1  = duty;
        TIM8->CCMR2 = CCMR2_REV_STATIC;
    }
    // CCR1 is preloaded (OC1PE) so without an update event the new phasing
    // lands up to half a PWM period after the CCMR2 force-mode flip, leaving
    // the A leg chopping at the old direction's duty meanwhile. Force the
    // transfer now; the chop-phase reset from UG is harmless twice per
    // electrical cycle and any OC1REF edge it causes still gets dead-time.
    TIM8->EGR = TIM_EGR_UG;
}

// Drive the H-bridge fully active (no PWM chop) in `dir`: the A leg's OC1REF is
// pinned to the active half for the whole cycle (CCR1 = 0 forward / = ARR
// reverse), so with the B leg on its static rail the winding sees the full rail
// voltage. Used by the pulse-width driver's ON compare; the OFF compare then
// drops MOE to float the winding. Kept in RAM alongside commutate_now.
__RAMFUNC__ static inline void drive_full(int8_t dir)
{
    s_direction = dir;
    if (dir > 0) {
        TIM8->CCR1  = 0;            // OC1REF never active → AH high all cycle
        TIM8->CCMR2 = CCMR2_FWD_STATIC;
    } else {
        TIM8->CCR1  = s_arr_v;      // OC1REF active all cycle → AL high all cycle
        TIM8->CCMR2 = CCMR2_REV_STATIC;
    }
    // CCR1 is preloaded (OC1PE): without an update event the write only lands
    // at the next overflow/underflow, up to half a PWM period away, while the
    // CCMR2 force-mode write above is immediate. Since pulse direction
    // alternates, the stale CCR1 is the opposite extreme — both A and B
    // terminals on the same rail, i.e. a winding short braking against the
    // back-EMF from the moment the caller asserts MOE until the transfer.
    // Force the transfer now, before MOE comes on. The counter reset from UG
    // is harmless here: OC1REF is pinned for the whole cycle either way.
    TIM8->EGR = TIM_EGR_UG;
}

// TIM5 compare ISR — fires at the scheduled ON/OFF instants of a drive pulse.
// CC1 = turn ON (drive full active in the latched direction, assert MOE);
// CC2 = turn OFF (drop MOE so the winding floats). Each match clears its own
// flag. The compares are inherently one-shot: with a free-running 32-bit
// counter a target is not revisited for ~27 s, so the channel simply waits,
// disarmed in effect, until the hall handler writes a fresh target. Both
// compare interrupts stay permanently enabled (armed/disarmed at the register
// level only by the main loop), so this ISR never touches DIER and cannot race
// the hall handler.
CH_IRQ_HANDLER(STM32_TIM5_HANDLER);
CH_IRQ_HANDLER(STM32_TIM5_HANDLER)
{
    CH_IRQ_PROLOGUE();
    const uint32_t sr = TIM5->SR & TIM5->DIER;
    if (sr & TIM_SR_CC1IF) {
        TIM5->SR = ~TIM_SR_CC1IF;
        drive_full(s_pulse_dir);
        TIM8->BDTR = BDTR_BASE | TIM_BDTR_MOE;   // H-bridge on
    }
    if (sr & TIM_SR_CC2IF) {
        TIM5->SR = ~TIM_SR_CC2IF;
        TIM8->BDTR = BDTR_BASE;                  // H-bridge off (winding floats)
    }
    if (sr & TIM_SR_CC3IF) {
        TIM5->SR = ~TIM_SR_CC3IF;
        SinglePhaseBLDC::sim_hall_edge_isr();    // simulated hall edge chain
    }
    CH_IRQ_EPILOGUE();
}

// Hall-edge EXTI handler — runs in GPIO interrupt context, once per hall flip.
// `pin_state` is the hall level latched at the edge and `now_us` is the
// interrupt timestamp in microseconds; both are supplied by the HAL functor
// dispatcher (pal_interrupt_cb_functor), so no register polling is needed here.
void SinglePhaseBLDC::hall_irq(uint8_t pin, bool pin_state, uint32_t now_us)
{
    // Simulation mode: the real hall sensor is disconnected from the control
    // path — only simulated edges (pin == SIM_HALL_FAKE_PIN) are processed.
    if (s_sim_active && pin != SIM_HALL_FAKE_PIN) {
        return;
    }

    // Inter-edge interval for RPM (two hall edges per electrical cycle).
    uint32_t dt = now_us - s_last_edge_us;
    if (dt > 0) {
        s_half_period_us = dt;
    }
    s_last_edge_us = now_us;

    // Pulse-width drive: schedule a single H-bridge pulse off this edge instead
    // of commutating inline. The half period (this edge to the previous one) is
    // averaged over the last SPEED_AVG_CYCLES edges; the pulse then opens
    // 1/4 pulse width (= 1/8 electrical cycle) after the edge, plus a fixed
    // offset, and stays on for throttle × 1/2 pulse width.
    if (s_pulse_en) {
        const uint32_t cnt        = TIM5->CNT;
        const uint32_t half_ticks = cnt - s_last_edge_ticks;   // 180° in TIM5 ticks

        // First edge after enabling only establishes the baseline tick count;
        // there is no measured half period to schedule from yet.
        if (!s_pulse_primed) {
            s_last_edge_ticks = cnt;
            s_pulse_primed    = true;
            return;
        }

        const uint32_t avg_prev = (s_half_ticks_cnt > 0)
                                ? (s_half_ticks_sum / s_half_ticks_cnt) : 0;

        // A half period far shorter than the average (< 1/4: the rotor cannot
        // quadruple its speed in one half cycle) is hall noise or contact
        // bounce, not rotation. Drop the edge outright: the average stays
        // clean, no full-rail pulse gets armed off a bounced level, and the
        // baseline is deliberately NOT advanced so the next genuine edge is
        // still measured from the last accepted one.
        if (avg_prev > 0 && half_ticks < (avg_prev >> 2)) {
            return;
        }

        s_last_edge_ticks = cnt;

        // A gap longer than ~2 electrical cycles (4 half periods) means the
        // motor had effectively stopped: discard the stale speed estimate and
        // treat this edge as a fresh baseline rather than driving on bad timing.
        if (avg_prev > 0 && half_ticks > (avg_prev << 2)) {
            s_half_ticks_sum = 0;
            s_half_ticks_idx = 0;
            s_half_ticks_cnt = 0;
            return;
        }

        // Roll the new half period into the SPEED_AVG_CYCLES running average.
        s_half_ticks_sum -= s_half_ticks_buf[s_half_ticks_idx];
        s_half_ticks_buf[s_half_ticks_idx] = half_ticks;
        s_half_ticks_sum += half_ticks;
        s_half_ticks_idx  = (uint8_t)((s_half_ticks_idx + 1U) % SPEED_AVG_CYCLES);
        if (s_half_ticks_cnt < SPEED_AVG_CYCLES) {
            s_half_ticks_cnt++;
        }
        const uint32_t avg_half = s_half_ticks_sum / s_half_ticks_cnt;

        // ON delay = 1/4 pulse width + fixed offset; ON duration = throttle ×
        // 1/2 pulse width, hard-capped at PULSE_MAX_ON_TICKS — the pulse is
        // full rail with no chop, so on a slow rotor (little back-EMF) a long
        // window means winding current limited only by resistance. Latch the
        // drive direction this edge commands.
        const uint32_t delay_ticks = (avg_half >> 2) + PULSE_DELAY_OFFSET_TICKS;
        uint32_t on_ticks = (uint32_t)(s_throttle_v * 0.5f * (float)avg_half);
        if (on_ticks > PULSE_MAX_ON_TICKS) {
            on_ticks = PULSE_MAX_ON_TICKS;
        }
        s_pulse_dir = hall_dir(pin_state);

        // Arm the ON (CC1) and OFF (CC2) compares as absolute targets and clear
        // any stale flags. Both compare interrupts are already enabled (see
        // update_pid), so no DIER read-modify-write here — nothing for the TIM5
        // ISR to race.
        const uint32_t t_on  = cnt + delay_ticks;
        TIM5->CCR1 = t_on;
        TIM5->SR   = ~TIM_SR_CC1IF;
        TIM5->CCR2 = t_on + on_ticks;
        TIM5->SR   = ~TIM_SR_CC2IF;
        return;
    }

    if (!s_commutate_en) {
        return;
    }

    // Immediate (startup) commutation, direction from the hall via hall_dir().
    commutate_now(hall_dir(pin_state));
}

// SBUS-edge EXTI handler — fires on both edges of the SBUS stream (PB13).
// `pin_state` is the level after the edge, so the elapsed time since the
// previous edge is the width of the state that just ended. A rising edge
// completes a (high, low) pair, which is queued for the main-loop decoder;
// widths are clamped to 16 bits (inter-frame idle gaps are much longer than
// anything the bit decoder needs to distinguish).
void SinglePhaseBLDC::sbus_irq(uint8_t pin, bool pin_state, uint32_t now_us)
{
    uint32_t width = now_us - s_sbus_edge_us;
    s_sbus_edge_us = now_us;
    if (width > 0xFFFFU) {
        width = 0xFFFFU;
    }

    if (!pin_state) {
        // falling edge: high state ended
        s_sbus_high_us = width;
        return;
    }

    // rising edge: low state ended — completes a (high, low) pair
    const uint16_t next = (s_sbus_head + 1U) & (SBUS_PULSE_BUF_SIZE - 1U);
    if (next != s_sbus_tail) {   // drop the pair if the buffer is full
        s_sbus_buf[s_sbus_head].high_us = (uint16_t)s_sbus_high_us;
        s_sbus_buf[s_sbus_head].low_us  = (uint16_t)width;
        s_sbus_head = next;
    }
}

// Drain queued pulse pairs into the SBUS decoder and latch the channel set
// whenever a complete frame has been decoded.
void SinglePhaseBLDC::update_sbus(void)
{
    auto &rcprot = AP::RC();

    while (s_sbus_tail != s_sbus_head) {
        const sbus_pulse &p = s_sbus_buf[s_sbus_tail];
        rcprot.process_pulse(p.high_us, p.low_us);
        s_sbus_tail = (s_sbus_tail + 1U) & (SBUS_PULSE_BUF_SIZE - 1U);
    }

    if (rcprot.new_input()) {
        _rc_num_channels = MIN(rcprot.num_channels(), MAX_RCIN_CHANNELS);
        rcprot.read(_rc_channels, _rc_num_channels);
        _rc_failsafe     = rcprot.failsafe_active();
        _rc_last_frame_ms = AP_HAL::millis();
    }
}

// Link health: frames still arriving and the receiver not flagging failsafe.
bool SinglePhaseBLDC::rc_link_ok(void) const
{
    if (_rc_num_channels == 0 || _rc_failsafe) {
        return false;
    }
    return AP_HAL::millis() - _rc_last_frame_ms < SBUS_TIMEOUT_MS;
}

// Failsafe: receiver flagging SBUS failsafe, or the link dropping out after
// having been up. A receiver that has never delivered a frame is "no RC",
// not failsafe, so the LED keeps its normal heartbeat on the bench.
bool SinglePhaseBLDC::rc_failsafe(void) const
{
    if (_rc_num_channels == 0) {
        return false;
    }
    return _rc_failsafe ||
           (AP_HAL::millis() - _rc_last_frame_ms >= SBUS_TIMEOUT_MS);
}

// Run request from the RC link: motor channel below the threshold.
bool SinglePhaseBLDC::rc_run_requested(void) const
{
    if (!rc_link_ok() || _rc_num_channels < SBUS_MOTOR_CHANNEL) {
        return false;
    }
    return _rc_channels[SBUS_MOTOR_CHANNEL - 1U] < SBUS_MOTOR_THRESHOLD_US;
}

// Throttle channel linearly mapped to duty:
// SBUS_THROTTLE_IN_MIN..IN_MAX µs → OUT_MIN..OUT_MAX, input clamped.
float SinglePhaseBLDC::rc_throttle(void) const
{
    const float v = constrain_float(_rc_channels[SBUS_THROTTLE_CHANNEL - 1U],
                                    SBUS_THROTTLE_IN_MIN, SBUS_THROTTLE_IN_MAX);
    const float frac = (v - SBUS_THROTTLE_IN_MIN) /
                       (float)(SBUS_THROTTLE_IN_MAX - SBUS_THROTTLE_IN_MIN);
    return SBUS_THROTTLE_OUT_MIN +
           frac * (SBUS_THROTTLE_OUT_MAX - SBUS_THROTTLE_OUT_MIN);
}

// ---------------------------------------------------------------------------
// TIM8 helpers
// ---------------------------------------------------------------------------

// Apply duty in [0, 1] — bypasses MAX_THROTTLE. Writes CCR1 + CCMR2 for the
// current s_direction and syncs ISR mirrors. Called both from the main loop
// and (effectively, inline) from the commutation ISR.
void SinglePhaseBLDC::apply_pwm(float duty)
{
    duty         = constrain_float(duty, 0.0f, 1.0f);
    _throttle    = duty;
    s_throttle_v = duty;
    s_arr_v      = _arr;

    uint32_t d = (uint32_t)(duty * _arr);
    if (s_direction > 0) {
        TIM8->CCR1  = _arr - d;
        TIM8->CCMR2 = CCMR2_FWD_STATIC;
    } else {
        TIM8->CCR1  = d;
        TIM8->CCMR2 = CCMR2_REV_STATIC;
    }
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

    // CH1: PWM mode 1 with CCR preload — A leg switches normally, CH1N is the
    //      dead-time-inserted complement (AH).
    // CH3: B leg static. Init to forward (BL on, BH off); the commutation
    //      handler flips between CCMR2_FWD_STATIC and CCMR2_REV_STATIC.
    TIM8->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM8->CCMR2 = CCMR2_FWD_STATIC;

    // Start in the freewheel half (zero net drive) — apply_pwm(0) below
    // sets CCR1 = ARR which keeps OC1REF asserted across the whole cycle.
    TIM8->CCR1 = arr;
    TIM8->CCR3 = 0;   // unused under force mode

    // All outputs active-high; predriver handles gate voltage conversion.
    // CC1P=0:  CH1 (AL) active-high; CC1NP=0: CH1N (AH) active-high complement.
    // CC3P=0:  CH3 (BL) active-high; CC3NP=0: CH3N (BH) active-high complement.
    TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC3E | TIM_CCER_CC3NE;

    TIM8->BDTR = BDTR_BASE;   // bridge off (MOE=0 until set_enabled(true))
    TIM8->EGR  = TIM_EGR_UG;
    TIM8->SR   = 0;
    TIM8->CR1 |= TIM_CR1_CEN;
}

// Free-running 32-bit time base for the one-shot commutation compares. PSC = 0
// (160 MHz, 6.25 ns/tick), ARR = full range so the counter never reloads — the
// commutation instants are absolute compare targets (CNT + delay). CC1/CC2 are
// left in frozen output-compare mode (CCMR1 = 0): no pin output, but a match
// still raises CCxIF and, when armed via DIER, the TIM5 interrupt. Compares are
// armed per-edge by the hall handler and disarmed in the ISR (one-shot).
static void tim5_init()
{
    rccEnableTIM5(true);

    TIM5->CR1  = 0;
    TIM5->CR2  = 0;
    TIM5->SMCR = 0;
    TIM5->DIER = 0;
    TIM5->CCMR1 = 0;          // CC1/CC2 = output compare, frozen
    TIM5->CCER  = 0;          // no physical outputs
    TIM5->PSC   = 0;
    TIM5->ARR   = 0xFFFFFFFFU;
    TIM5->CCR1  = 0;
    TIM5->CCR2  = 0;
    TIM5->EGR   = TIM_EGR_UG;
    TIM5->SR    = 0;
    TIM5->CR1  |= TIM_CR1_CEN;

    nvicEnableVector(STM32_TIM5_NUMBER, CORTEX_MAX_KERNEL_PRIORITY);
}

// ---------------------------------------------------------------------------
// Motor-lead voltage sense (ADC2, polled)
//
// PA4 (ADC2_IN17) and PA7 (ADC2_IN4) each tap a motor lead through a 49.9k/2.2k
// divider. The ardupilot HAL ADC driver only supports ADC1 on the STM32G4, so
// ADC2 is driven directly here, matching the register-level style used for the
// TIM8 setup above. Conversions are software-triggered and polled — no DMA
// or interrupts — which is plenty for an occasional debug readout.
// ---------------------------------------------------------------------------

static void adc2_init()
{
    // Motor-lead sense pins to analog mode (disconnect the digital buffers).
    palSetLineMode(PAL_LINE(GPIOA, 4U), PAL_MODE_INPUT_ANALOG);   // PA4 = lead A
    palSetLineMode(PAL_LINE(GPIOA, 7U), PAL_MODE_INPUT_ANALOG);   // PA7 = lead B

    rccEnableADC12(true);   // ADC1/2 peripheral clock

    // Synchronous clock, HCLK/4 (= 40 MHz @ 160 MHz HCLK). Set while the ADCs
    // are disabled; lives in the shared ADC12 common register.
    ADC12_COMMON->CCR = (ADC12_COMMON->CCR & ~ADC_CCR_CKMODE_Msk) |
                        ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1;

    // Leave deep power-down and turn on the internal voltage regulator.
    ADC2->CR &= ~ADC_CR_DEEPPWD;
    ADC2->CR |= ADC_CR_ADVREGEN;
    hal.scheduler->delay_microseconds(20);   // T_ADCVREG_STUP (max 20 us)

    // Single-ended calibration (must run with ADEN = 0).
    ADC2->CR &= ~ADC_CR_ADCALDIF;
    ADC2->CR |= ADC_CR_ADCAL;
    while (ADC2->CR & ADC_CR_ADCAL) { }

    // 12-bit, right-aligned, single conversion (CONT = 0).
    ADC2->CFGR = 0;

    // Long sample time (640.5 cycles, SMP = 7) on both channels — the divider's
    // ~2 k output impedance needs a generous acquisition window.
    ADC2->SMPR1 = (7U << ADC_SMPR1_SMP4_Pos);    // channel 4  (PA7)
    ADC2->SMPR2 = (7U << ADC_SMPR2_SMP17_Pos);   // channel 17 (PA4)

    // Enable and wait until ready.
    ADC2->ISR = ADC_ISR_ADRDY;
    ADC2->CR |= ADC_CR_ADEN;
    while (!(ADC2->ISR & ADC_ISR_ADRDY)) { }
    ADC2->ISR = ADC_ISR_ADRDY;
}

// Single polled conversion of one ADC2 channel → raw 12-bit count.
static uint16_t adc2_sample(uint8_t chan)
{
    ADC2->SQR1 = (uint32_t)chan << ADC_SQR1_SQ1_Pos;   // L=0 (1 conv), SQ1=chan
    ADC2->ISR  = ADC_ISR_EOC;
    ADC2->CR  |= ADC_CR_ADSTART;
    while (!(ADC2->ISR & ADC_ISR_EOC)) { }
    return (uint16_t)(ADC2->DR & ADC_DR_RDATA);        // reading DR clears EOC
}

// Read one motor-lead channel and return the voltage corrected for the divider.
static float adc2_read_lead_volts(uint8_t chan)
{
    const float pin_v = adc2_sample(chan) *
                        (MOTOR_SENSE_ADC_VREF / MOTOR_SENSE_ADC_FULL);
    return pin_v * MOTOR_SENSE_DIV_RATIO;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void SinglePhaseBLDC::init()
{
    s_sim_owner = this;   // lets the TIM5 CC3 ISR feed simulated hall edges

    _arr    = TIM8_CLK_HZ / (2U * DEFAULT_FREQ_HZ);
    s_arr_v = _arr;
    tim8_init(_arr);
    tim5_init();   // 32-bit time base for phase-delayed commutation
    apply_pwm(0.0f);

    // Hall sensor drives commutation directly via a GPIO edge interrupt — the
    // handler fires on every flip (both edges) and timestamps it for RPM.
    hal.gpio->attach_interrupt(
        GPIO_MOTOR_PHASE,
        FUNCTOR_BIND_MEMBER(&SinglePhaseBLDC::hall_irq, void, uint8_t, bool, uint32_t),
        AP_HAL::GPIO::INTERRUPT_BOTH);

    // SBUS RC input — software-decoded from edge timestamps (PB13 has no
    // USART RX function). Only the SBUS backend is compiled in (see hwdef);
    // mask bit 0 = "all protocols" so no narrowing is needed here.
    auto &rcprot = AP::RC();
    rcprot.init();
    rcprot.set_rc_protocols(1);
    hal.gpio->attach_interrupt(
        GPIO_SBUS_IN,
        FUNCTOR_BIND_MEMBER(&SinglePhaseBLDC::sbus_irq, void, uint8_t, bool, uint32_t),
        AP_HAL::GPIO::INTERRUPT_BOTH);

    adc2_init();   // PA4/PA7 motor-lead voltage sense

    //set_test_mode(true);
}

void SinglePhaseBLDC::set_enabled(bool en)
{
    _enabled   = en;
    TIM8->BDTR = en ? (BDTR_BASE | TIM_BDTR_MOE) : BDTR_BASE;
}

void SinglePhaseBLDC::set_throttle(float throttle)
{
    throttle = constrain_float(throttle, 0.0f, MAX_THROTTLE);
    if (s_pulse_en) {
        // Pulse-width drive owns the TIM8 CCR1/CCMR2 phasing and MOE (the TIM5
        // ISR sets them at each pulse). Writing those registers from here would
        // race that ISR. Throttle only sets the pulse width, so just hand the
        // new value to the ISR via the mirror; it takes effect on the next hall
        // edge (well under a millisecond away in PID mode).
        _throttle    = throttle;
        s_arr_v      = _arr;
        s_throttle_v = throttle;
        return;
    }
    apply_pwm(throttle);
}

void SinglePhaseBLDC::set_frequency(uint32_t hz)
{
    _arr      = TIM8_CLK_HZ / (2U * hz);
    TIM8->ARR = _arr - 1;
    apply_pwm(_throttle);
    TIM8->EGR = TIM_EGR_UG;
    TIM8->SR  = 0;
}

void SinglePhaseBLDC::flip_direction()
{
    // Disable bridge during reconfiguration, swap phase, re-apply duty.
    // Caller is responsible for re-enabling.
    TIM8->BDTR = BDTR_BASE;
    s_direction = -s_direction;
    apply_pwm(_throttle);
    TIM8->EGR  = TIM_EGR_UG;
    TIM8->SR   = 0;
}

// ---------------------------------------------------------------------------
// RPM (called from main loop — reads ISR volatile state)
// ---------------------------------------------------------------------------

void SinglePhaseBLDC::compute_rpm()
{
    // Snapshot volatile ISR state; minor race is acceptable (one-edge error max)
    uint32_t half_us = s_half_period_us;
    uint32_t age_us  = AP_HAL::micros() - s_last_edge_us;

    if (half_us == 0 || age_us >= RPM_TIMEOUT_US) {
        _electrical_rpm = 0.0f;
        return;
    }
    // Two hall edges per electrical cycle → full electrical period = 2 × half
    float electrical_period_s = 2.0f * half_us * 1e-6f;
    _electrical_rpm = 60.0f / electrical_period_s;
}

// ---------------------------------------------------------------------------
// Mode state machines (called from main loop at ~1 kHz)
// ---------------------------------------------------------------------------

void SinglePhaseBLDC::update_startup()
{
    const uint32_t now = AP_HAL::millis();

    // First entry: latch initial direction from hall (pulls toward the next
    // commutation point) and start the first pulse.
    if (_kick_start_ms == 0) {
        _kick_start_ms       = now;
        _kick_phase_start_ms = now;
        _kick_resting        = false;
        _commute_active      = false;
        _kick_attempts       = 0;
        s_commutate_en       = false;
        s_direction          = hall_dir(read_hall());
        // Raw level (not direction) — used only to detect that the rotor moved,
        // so it is independent of HALL_POLARITY.
        _hall_at_kick_start  = read_hall();
    }

    // Phase 2: rotor confirmed moving → ISR drives commutation, hold throttle
    // until we reach the RPM threshold for PID hand-off.
    if (_commute_active) {
        s_commutate_en = true;
        apply_pwm(STARTUP_COMMUTE_THROTTLE);
        set_enabled(true);

        if (_electrical_rpm >= STARTUP_RPM_THRESHOLD) {
            _mode          = DriveMode::PID;
            _pid_integral  = 0.0f;
            _pid_last_err  = TARGET_RPM - _electrical_rpm;
            _pid_last_us   = AP_HAL::micros();
        }
        return;
    }

    // Phase 1b: bridge-off rest between pulses. Lets current decay before the
    // next attempt. Direction is NOT flipped — every retry pushes the same
    // way, picked once from the hall reading at startup entry.
    if (_kick_resting) {
        set_enabled(false);
        apply_pwm(0.0f);
        if (now - _kick_phase_start_ms >= STARTUP_REST_MS) {
            _kick_phase_start_ms = now;
            _kick_resting        = false;
            _kick_attempts++;
            if (_kick_attempts >= STARTUP_MAX_KICKS) {
                // Rotor never moved — bail out so the operator notices.
                _mode = DriveMode::STOPPING;
            }
        }
        return;
    }

    // Phase 1a: active kick pulse. Drive at fixed polarity for the full
    // STARTUP_KICK_MS — the hall sensor is intentionally NOT polled here so
    // a noisy edge mid-pulse can't yank us into commutation before the kick
    // has had a chance to do its job.
    s_commutate_en = false;
    apply_pwm(STARTUP_KICK_THROTTLE);
    set_enabled(true);

    if (now - _kick_phase_start_ms >= STARTUP_KICK_MS) {
        // Pulse complete — only now is the hall sampled to decide success.
        if (read_hall() != _hall_at_kick_start) {
            _commute_active = true;     // rotor moved → hand off to ISR
            return;
        }
        _kick_resting        = true;    // no movement → rest, then retry
        _kick_phase_start_ms = now;
    }
}

void SinglePhaseBLDC::update_pid()
{
    // First PID tick: hand commutation from the immediate startup path over to
    // the pulse-width TIM5 scheduler. The bridge is dropped here so the pulse
    // scheduler (not the main loop) owns MOE from now on; the first hall edge
    // re-bases the tick count and the speed average before any pulse is armed.
    if (!s_pulse_en) {
        s_commutate_en   = false;          // stop immediate commutation
        s_pulse_primed   = false;          // first edge re-bases the tick count
        s_half_ticks_sum = 0;              // restart the speed average
        s_half_ticks_idx = 0;
        s_half_ticks_cnt = 0;
        TIM8->BDTR = BDTR_BASE;            // bridge off; TIM5 scheduler owns MOE
        // Arm both TIM5 compare interrupts (clear stale flags first). They stay
        // enabled for the whole PID run; the hall handler only writes targets.
        // OR rather than assign: CC3IE belongs to the simulated-hall edge chain
        // and must survive a PID entry while simulating.
        TIM5->SR    = ~(TIM_SR_CC1IF | TIM_SR_CC2IF);
        TIM5->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;
        s_pulse_en = true;
    }
    _enabled = true;   // armed for the LED; the TIM5 scheduler pulses MOE

    // No hall edge for a full second → the motor has stalled; give up and let
    // the main loop drop back to IDLE. (The single-shot pulses already leave
    // the bridge disabled once edges stop, so nothing is being driven here.)
    if (AP_HAL::micros() - s_last_edge_us >= PULSE_STOP_TIMEOUT_US) {
        _mode = DriveMode::STOPPING;
        return;
    }

    // RC link up → throttle channel commands the pulse width directly, no RPM
    // PID. Keep _pid_last_us current so dt stays sane if the link drops and the
    // PID takes back over.
    if (rc_link_ok() && _rc_num_channels >= SBUS_THROTTLE_CHANNEL) {
        set_throttle(rc_throttle());
        _pid_last_us = AP_HAL::micros();
    } else {
        uint32_t now_us = AP_HAL::micros();
        float dt = (now_us - _pid_last_us) * 1e-6f;
        _pid_last_us = now_us;

        if (dt > 0.0f && dt <= 0.5f) {
            float err      = TARGET_RPM - _electrical_rpm;
            _pid_integral += err * dt;
            _pid_integral  = constrain_float(_pid_integral,
                                             -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
            float deriv    = (err - _pid_last_err) / dt;
            _pid_last_err  = err;

            float duty = PID_KP * err + PID_KI * _pid_integral + PID_KD * deriv;
            set_throttle(duty);  // public setter applies the MAX_THROTTLE safety cap
        }
    }
}

void SinglePhaseBLDC::update_stopping()
{
    // Disarm the pulse scheduler before dropping the bridge so a pending TIM5
    // compare can't re-enable the H-bridge after it is disabled. Clear only the
    // pulse compares: CC3IE is the simulated-hall edge chain, which must keep
    // running through a stop while simulating. All DIER writers are main-loop
    // context, so the read-modify-write cannot race.
    s_pulse_en  = false;
    TIM5->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_CC2IE);
    apply_pwm(0.0f);
    set_enabled(false);
    s_commutate_en       = false;
    _mode                = DriveMode::IDLE;
    _commute_active      = false;
    _kick_resting        = false;
    _kick_start_ms       = 0;
    _kick_phase_start_ms = 0;
    _kick_attempts       = 0;
    _pid_integral        = 0.0f;
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
        apply_pwm(MAX_THROTTLE);
    }
    _prev_deadman = deadman;

    if (deadman) {
        set_enabled(true);
    } else {
        set_enabled(false);
        apply_pwm(0.0f);
        _mode = DriveMode::IDLE;
    }
}

// ---------------------------------------------------------------------------
// Simulation mode (SBUS channel 5 high)
//
// The control stack runs completely unchanged — the only substitution is the
// hall data source. A simple rotor model supplies the fake hall edges:
//   - 4 pole pairs / 4 hall cycles per mechanical rev → 8 edges of 45° each
//     (matching the 1/8-turn cogging detents of the real motor),
//   - steady-state speed proportional to throttle (69600 mech RPM at ~50%),
//   - first-order spin-up (τ = SIM_ACCEL_TAU_S, ~200 ms to 60k RPM),
//   - constant-deceleration coast (~500 ms from full speed).
// Fast edges (> SIM_ISR_HANDOFF_RPM) are chained by the TIM5 CC3 compare ISR;
// slow ones are emitted by the 1 kHz main loop from the integrated angle, so
// the kick-by-kick startup sequence sees realistic partial rotations.
// ---------------------------------------------------------------------------

// Hall level for the control logic: simulated while sim is active, real pin
// otherwise. hall_irq() applies the matching gate on the interrupt path.
bool SinglePhaseBLDC::read_hall(void) const
{
    if (s_sim_active) {
        return s_sim_hall;
    }
    return palReadLine(HALL_LINE) != 0;
}

// TIM5 CC3 compare ISR body — one simulated hall edge. Chains the next edge
// first (minimum jitter), then toggles the fake hall level and hands it to the
// stock hall handler exactly as the EXTI would, just with the fake pin id.
void SinglePhaseBLDC::sim_hall_edge_isr(void)
{
    const uint32_t half = s_sim_half_ticks;
    if (!s_sim_active || half == 0 || s_sim_owner == nullptr) {
        // Sim ended or the main loop took edge generation back: let the chain
        // die. The main loop re-arms CC3 when it hands the edges back.
        s_sim_isr_armed = false;
        return;
    }
    TIM5->CCR3 = TIM5->CNT + half;
    sim_hall_set(!s_sim_hall);
    s_sim_edge_count++;
    s_sim_owner->hall_irq(SIM_HALL_FAKE_PIN, s_sim_hall, AP_HAL::micros());
}

void SinglePhaseBLDC::update_simulation(bool requested)
{
    const uint32_t now_us = AP_HAL::micros();

    if (requested != _sim_active) {
        // Entering or leaving simulation: put the drive into a clean stop and
        // reset the rotor model. Half ticks are zeroed before touching DIER so
        // a CC3 already in flight sees a dead chain and exits harmlessly.
        if (_mode == DriveMode::STARTUP || _mode == DriveMode::PID ||
            _mode == DriveMode::TEST) {
            _mode = DriveMode::STOPPING;
        }
        _sim_mech_rpm    = 0.0f;
        _sim_angle_deg   = 0.0f;
        _sim_edge_accum  = 0.0f;
        _sim_fault_ms    = 0;
        _sim_last_us     = now_us;
        s_sim_half_ticks = 0;
        s_sim_edge_count = 0;
        // Stopped rotor: the hall can rest at either level — seed from the
        // real pin so the sim starts from whatever the sensor happens to show.
        sim_hall_set(palReadLine(HALL_LINE) != 0);
        if (requested) {
            TIM5->SR    = ~TIM_SR_CC3IF;
            TIM5->DIER |= TIM_DIER_CC3IE;
        } else {
            TIM5->DIER &= ~TIM_DIER_CC3IE;
            s_sim_isr_armed = false;
        }
        s_sim_active = requested;
        _sim_active  = requested;
        hal.console->printf("SIM: %s\n",
                            requested ? "ENABLED — hall simulated, H-bridge LIVE"
                                      : "disabled — real hall sensor active");
    }

    if (!_sim_active) {
        return;
    }

    const float dt = (now_us - _sim_last_us) * 1e-6f;
    _sim_last_us = now_us;
    if (dt <= 0.0f || dt > 0.05f) {
        return;   // first tick after a stall of the loop — skip integration
    }

    // Rotor model. "Driving" mirrors what the H-bridge is actually doing:
    // bridge armed with a nonzero commanded duty/pulse window. Startup rest
    // gaps and the STOPPING/IDLE states coast.
    const bool driving = _enabled && s_throttle_v > 0.01f;
    if (driving) {
        const float target = s_throttle_v * SIM_RPM_PER_THROTTLE;
        _sim_mech_rpm += (target - _sim_mech_rpm) * (dt / SIM_ACCEL_TAU_S);
    } else if (_sim_mech_rpm > 0.0f) {
        _sim_mech_rpm -= SIM_DECEL_RPM_PER_S * dt;
        if (_sim_mech_rpm < 0.0f) {
            _sim_mech_rpm = 0.0f;
        }
    }

    if (_sim_mech_rpm >= SIM_ISR_HANDOFF_RPM) {
        // Fast regime: CC3 chains the edges. Edge interval = 1/8 mechanical
        // rev = 60e6/(8·rpm) µs = 7.5e6/rpm µs; the ISR reads the fresh value
        // at every edge, so acceleration tracks within one edge period.
        const float half_us = 7.5e6f / _sim_mech_rpm;
        s_sim_half_ticks = (uint32_t)(half_us * (float)(TIM5_CLK_HZ / 1000000UL));
        if (!s_sim_isr_armed) {
            s_sim_isr_armed = true;
            TIM5->SR   = ~TIM_SR_CC3IF;
            TIM5->CCR3 = TIM5->CNT + s_sim_half_ticks;
        }
        // Angle bookkeeping follows the edges the ISR actually generated.
        _sim_angle_deg  = (float)((s_sim_edge_count & 7U) * 45U);
        _sim_edge_accum = 0.0f;
    } else {
        // Slow regime: kill the ISR chain and emit edges from the integrated
        // angle once the pending compare has died (avoids double generators).
        // Worst case here is one edge per tick — plenty below the handoff
        // speed, where edges are at most one per 2.5 ms.
        s_sim_half_ticks = 0;
        if (!s_sim_isr_armed) {
            const float step = _sim_mech_rpm * 6.0f * dt;   // RPM → deg/s → deg
            _sim_angle_deg += step;
            if (_sim_angle_deg >= 360.0f) {
                _sim_angle_deg -= 360.0f;
            }
            _sim_edge_accum += step;
            if (_sim_edge_accum >= 45.0f) {
                _sim_edge_accum -= 45.0f;
                sim_hall_set(!s_sim_hall);
                s_sim_edge_count++;
                hall_irq(SIM_HALL_FAKE_PIN, s_sim_hall, now_us);
            }
        }
    }
}

static const char *mode_name(SinglePhaseBLDC::DriveMode m)
{
    switch (m) {
    case SinglePhaseBLDC::DriveMode::IDLE:     return "IDLE";
    case SinglePhaseBLDC::DriveMode::STARTUP:  return "STARTUP";
    case SinglePhaseBLDC::DriveMode::PID:      return "PID";
    case SinglePhaseBLDC::DriveMode::STOPPING: return "STOPPING";
    case SinglePhaseBLDC::DriveMode::TEST:     return "TEST";
    }
    return "?";
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

    // Decode any SBUS pulses captured since the last tick
    update_sbus();

    // RC failsafe → hold the blue ACT LED solid. AP_Periph::update() toggles
    // it as a 1 Hz heartbeat; rewriting it every 1 kHz tick overrides that
    // (the heartbeat resumes on its own once failsafe clears).
    if (rc_failsafe()) {
        palWriteLine(HAL_GPIO_PIN_LED, HAL_LED_ON);
    }

    // Simulation request: SBUS channel 5 high. The engine handles entry/exit
    // transitions itself and is a no-op while inactive.
    const bool sim_req = rc_link_ok() && _rc_num_channels >= SBUS_SIM_CHANNEL &&
                         _rc_channels[SBUS_SIM_CHANNEL - 1U] > SBUS_SIM_THRESHOLD_US;
    update_simulation(sim_req);

    // Low = pressed
    bool deadman = (hal.gpio->read(GPIO_DEADMAN_BUTTON) == 0);

    // Run request: deadman button OR'd with the SBUS motor channel
    bool run = deadman || rc_run_requested();

#if AP_PERIPH_BATTERY_ENABLED
    // Simulation-only protection: the bridge is firing with (supposedly) no
    // motor attached, so any real battery current gulp means a motor was left
    // connected and just started. Stop and lock the drive out for
    // SIM_FAULT_DISABLE_MS. Normal (non-sim) operation is untouched.
    if (_sim_active && _sim_fault_ms == 0 && periph.battery_lib.healthy(0)) {
        float amps;
        if (periph.battery_lib.current_amps(amps, 0) &&
            amps > SIM_FAULT_CURRENT_A) {
            _sim_fault_ms = AP_HAL::millis();
            hal.console->printf("SIM FAULT: %.2f A current gulp — motor connected? "
                                "Drive disabled for %u ms\n",
                                (double)amps, (unsigned)SIM_FAULT_DISABLE_MS);
        }
    }
    if (_sim_fault_ms != 0 &&
        AP_HAL::millis() - _sim_fault_ms >= SIM_FAULT_DISABLE_MS) {
        _sim_fault_ms = 0;
        hal.console->printf("SIM: fault lockout cleared\n");
    }
#endif
    if (_sim_fault_ms != 0) {
        run = false;   // forces STOPPING below and blocks IDLE → run
        if (_mode == DriveMode::TEST) {
            _mode = DriveMode::STOPPING;   // TEST follows the button directly
        }
    }

    compute_rpm();

    // Both run sources released mid-run → coast to stop
    if (!run && (_mode == DriveMode::STARTUP || _mode == DriveMode::PID)) {
        _mode = DriveMode::STOPPING;
    }

    switch (_mode) {
    case DriveMode::IDLE:
        // Wait for a run request; test mode bypasses the normal startup sequence
        if (run) {
            _mode = _test_mode_active ? DriveMode::TEST : DriveMode::STARTUP;
        }
        break;
    case DriveMode::STARTUP:  update_startup();  break;
    case DriveMode::PID:      update_pid();      break;
    case DriveMode::STOPPING: update_stopping(); break;
    case DriveMode::TEST:     update_test();     break;
    }

    // Right LED solid whenever the H-bridge is energised; rapid flash
    // (~15 Hz) while a simulation current fault has the drive locked out
    if (_sim_fault_ms != 0) {
        blink_led(((AP_HAL::millis() >> 5) & 1U) != 0);
    } else {
        blink_led(_enabled);
    }

#if AP_PERIPH_BATTERY_ENABLED
    const uint32_t now_ms = AP_HAL::millis();

    const float mech_rpm = _electrical_rpm / (float)POLE_PAIRS;
    if (mech_rpm > _peak_mech_rpm) {
        _peak_mech_rpm = mech_rpm;
        // Sample both motor leads at the new RPM peak (already divider-corrected).
        _vsense_a_at_peak = adc2_read_lead_volts(MOTOR_SENSE_A_ADC2_CHAN);
        _vsense_b_at_peak = adc2_read_lead_volts(MOTOR_SENSE_B_ADC2_CHAN);
    }
    if (_throttle > _peak_throttle) {
        _peak_throttle = _throttle;
    }
    if (periph.battery_lib.healthy(0)) {
        float current;
        if (periph.battery_lib.current_amps(current, 0) && current > _peak_current) {
            _peak_current = current;
            _volt_at_peak_current = periph.battery_lib.voltage(0);
        }
    }

    if (now_ms - _batt_print_ms >= 200) {
        _batt_print_ms = now_ms;
        hal.console->printf("Mode: %-8s kicks=%u  Mech RPM: %.1f  Thr: %.3f  I: %.3fA  V: %.2fV  RC ch%u: %u(%s)  thr ch%u: %.2f\n",
                            mode_name(_mode),
                            (unsigned)_kick_attempts,
                            (double)_peak_mech_rpm,
                            (double)_peak_throttle,
                            (double)_peak_current,
                            (double)_volt_at_peak_current,
                            (unsigned)SBUS_MOTOR_CHANNEL,
                            (unsigned)_rc_channels[SBUS_MOTOR_CHANNEL - 1U],
                            rc_run_requested() ? "ON" : "off",
                            (unsigned)SBUS_THROTTLE_CHANNEL,
                            (double)rc_throttle());
        if (_rc_num_channels == 0) {
            hal.console->printf("    SBUS: no signal\n");
        }
        if (_sim_active) {
            hal.console->printf("    SIM: angle %5.1f deg  mech rpm %6.0f  hall %c%s\n",
                                (double)_sim_angle_deg,
                                (double)_sim_mech_rpm,
                                s_sim_hall ? 'H' : 'L',
                                _sim_fault_ms != 0 ? "  ** CURRENT FAULT — LOCKED OUT **"
                                                   : "");
        }
        hal.console->printf("    Motor-lead @ peak RPM:  A: %.2fV  B: %.2fV  diff: %.2fV\n",
                            (double)_vsense_a_at_peak,
                            (double)_vsense_b_at_peak,
                            (double)(_vsense_a_at_peak - _vsense_b_at_peak));
        _peak_mech_rpm        = 0.0f;
        _peak_throttle        = 0.0f;
        _peak_current         = 0.0f;
        _volt_at_peak_current = 0.0f;
        _vsense_a_at_peak     = 0.0f;
        _vsense_b_at_peak     = 0.0f;
    }
#endif
}

void SinglePhaseBLDC::blink_led(bool on)
{
    hal.gpio->write(GPIO_TEST_LED, on);
}

#endif
