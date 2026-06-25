#pragma once

#if AP_PERIPH_SINGLE_PHASE_BLDC

#include <AP_RCProtocol/AP_RCProtocol.h>   // MAX_RCIN_CHANNELS

// --- Tunable parameters ---------------------------------------------------
// Throttle is unipolar PWM duty cycle [0, 1]:
//   0 = synchronous freewheel (zero net drive)
//   1 = full active drive in the hall-selected direction
// STARTUP_* values bypass MAX_THROTTLE so the spin-up sequence can momentarily
// push past the runtime safety cap.
// 1.0 = DC kick (CCR1 saturates so OC1REF never transitions → bridge held
// in steady active state for the full pulse, no 20 kHz chopping). Lower
// values current-limit via PWM but you'll see the switching on a scope.
#define STARTUP_KICK_THROTTLE    0.25f      // duty during each kick pulse
#define STARTUP_KICK_MS          5U       // kick pulse duration (ms)
#define STARTUP_REST_MS          10U       // bridge-off gap between pulses (ms)
#define STARTUP_MAX_KICKS        50U       // give up if rotor never moves
#define STARTUP_COMMUTE_THROTTLE 0.25f      // duty during hall-commuted spin-up
#define STARTUP_RPM_THRESHOLD    500.0f    // electrical RPM to enter PID mode

#define TARGET_RPM               500000.0f // electrical RPM PID setpoint
#define PID_KP                   0.002f
#define PID_KI                   0.0001f
#define PID_KD                   0.0f
#define PID_INTEGRAL_MAX         5.0f      // anti-windup clamp

#define POLE_PAIRS               4U        // mechanical RPM = electrical RPM / POLE_PAIRS
#define RPM_TIMEOUT_US           500000U   // no hall edge for this long → RPM = 0

// SBUS RC input (PB13 / GPIO_SBUS_IN): inverted 100 kbaud serial from the
// receiver. PB13 has no USART RX or timer-capture function on the G474, so
// the stream is decoded in software: the GPIO EXTI handler timestamps every
// edge and queues (high, low) pulse-width pairs; the main loop feeds them to
// AP_RCProtocol, whose SoftSerial decoder reconstructs the SBUS frames.
// The motor channel < threshold = run request, OR'd with the deadman
// button. While the link is up the throttle channel commands the duty
// directly (linear map, input clamped to the IN_MIN..IN_MAX range); without
// RC the RPM PID takes over. A stale link (no decoded frame within the
// timeout) or an SBUS failsafe flag reads as off so a dead link can't run
// the motor.
#define SBUS_MOTOR_CHANNEL       7U        // 1-based channel that runs the motor
#define SBUS_MOTOR_THRESHOLD_US  1200U     // channel below this = run
#define SBUS_THROTTLE_CHANNEL    3U        // 1-based channel commanding duty
#define SBUS_THROTTLE_IN_MIN     1000U     // µs mapped to OUT_MIN duty
#define SBUS_THROTTLE_IN_MAX     2000U     // µs mapped to OUT_MAX duty
#define SBUS_THROTTLE_OUT_MIN    0.10f
#define SBUS_THROTTLE_OUT_MAX    0.80f
#define SBUS_TIMEOUT_MS          200U      // no frame for this long → off

// Motor-lead voltage sense divider: each lead taps the ADC pin through a 49.9k
// (top) / 2.2k (bottom) divider. The ADC measures the divided pin voltage;
// multiply by this ratio to recover the actual motor-lead voltage.
#define MOTOR_SENSE_R_TOP        49900.0f
#define MOTOR_SENSE_R_BOTTOM     2200.0f
#define MOTOR_SENSE_DIV_RATIO    ((MOTOR_SENSE_R_TOP + MOTOR_SENSE_R_BOTTOM) / MOTOR_SENSE_R_BOTTOM)

// ADC2 channels for the two motor-lead sense pins (STM32G474):
//   PA4 = ADC2_IN17 (lead A),  PA7 = ADC2_IN4 (lead B).
#define MOTOR_SENSE_A_ADC2_CHAN  17U
#define MOTOR_SENSE_B_ADC2_CHAN  4U
// ADC reference (VDDA) and full-scale count for the 12-bit conversions.
#define MOTOR_SENSE_ADC_VREF     3.3f
#define MOTOR_SENSE_ADC_FULL     4096.0f

// Runtime safety cap applied by set_throttle(). With 5 µs dead-time on a
// 50 µs PWM period the usable upper bound for full active drive is ~0.8.
#define MAX_THROTTLE             0.8f

// Hall polarity relative to the H-bridge: the single place that maps the hall
// pin level to the drive direction. With +1 a high hall level drives forward
// (CC1); with -1 it drives reverse (CC2). Flip this one value if the sensor is
// mounted or wired the other way round and the motor commutates backwards —
// every consumer (startup kick, immediate commutation, and the phase-delayed
// scheduler) routes through hall_dir(), so nothing else needs touching.
#define HALL_POLARITY            (+1)

// --- Commutation phase optimisation (perturb & observe) -------------------
// The hall sensor marks a fixed electrical position, but its mechanical
// mounting is rarely aligned with the optimum commutation instant. Rather
// than commutate the instant a hall edge arrives (phase = 0, as the bare
// driver did), each edge arms a one-shot timer; commutation happens when the
// timer expires, _phase_deg electrical degrees away from the edge. Positive
// = retard (commutate after the edge), negative = advance (before it — the
// scheduler folds a negative offset forward by one electrical period so the
// commutation lands just before the *next* same-polarity edge).
//
// _phase_deg is tuned online by a perturb-and-observe hill climb: every
// PHASE_PO_INTERVAL_MS the objective is measured and the offset nudged one
// step; if the objective got worse the search direction reverses. The
// objective is electrical RPM when the throttle is externally fixed (RC
// throttle: maximise speed) and negative duty when the RPM PID is regulating
// (minimise the drive needed to hold the setpoint).
#define PHASE_PO_STEP_DEG        2.0f      // perturbation per P&O step (elec deg)
#define PHASE_PO_MIN_DEG       (-45.0f)    // search lower bound
#define PHASE_PO_MAX_DEG         45.0f     // search upper bound
#define PHASE_PO_INTERVAL_MS     150U      // settle + measure window per step
#define PHASE_PO_RPM_DEADBAND    5.0f      // elec-RPM change treated as noise
#define PHASE_PO_DUTY_DEADBAND   0.003f    // duty change treated as noise
#define PHASE_PO_THR_STABLE      0.02f     // duty move that invalidates an RPM compare
#define PHASE_MIN_DELAY_US       3U        // floor for the one-shot timer delay
// --------------------------------------------------------------------------

class SinglePhaseBLDC {
public:
    friend class AP_Periph_FW;

    enum class DriveMode : uint8_t { IDLE, STARTUP, PID, STOPPING, TEST };

    void  update(void);
    void  init(void);

    void  set_enabled(bool en);
    void  set_throttle(float throttle);          // clamps to [0, MAX_THROTTLE]
    void  flip_direction(void);
    void  set_frequency(uint32_t hz);
    float get_rpm(void) const {                  // mechanical (shaft) RPM
        return _electrical_rpm / (float)POLE_PAIRS;
    }

    // Enable H-bridge test mode: hold button = bridge on, release = off,
    // each press flips direction. Overrides normal STARTUP/PID flow.
    void  set_test_mode(bool en) { _test_mode_active = en; }

private:
    void apply_pwm(float duty);   // [0, 1] — writes CCR1 + CCMR2, no safety cap
    void compute_rpm(void);

    // Hall-edge EXTI handler (runs in GPIO interrupt context). Commutates the
    // H-bridge on every hall flip and timestamps the edge for RPM estimation.
    void hall_irq(uint8_t pin, bool pin_state, uint32_t timestamp_us);

    // SBUS-edge EXTI handler (runs in GPIO interrupt context). Timestamps
    // every edge of the SBUS stream and queues (high, low) pulse-width pairs
    // for the main-loop decoder.
    void sbus_irq(uint8_t pin, bool pin_state, uint32_t timestamp_us);

    // Drain queued pulse pairs into AP_RCProtocol and latch decoded channels.
    void update_sbus(void);

    // True while frames are arriving and the receiver is not in failsafe.
    bool rc_link_ok(void) const;

    // True when the receiver flags SBUS failsafe, or the link has gone
    // stale after having been up. Drives the solid-LED indication.
    bool rc_failsafe(void) const;

    // True while the motor channel reads below SBUS_MOTOR_THRESHOLD_US and
    // the link is OK.
    bool rc_run_requested(void) const;

    // Throttle channel mapped to [SBUS_THROTTLE_OUT_MIN, _OUT_MAX] duty.
    // Only meaningful while rc_link_ok() is true.
    float rc_throttle(void) const;

    void update_startup(void);
    void update_pid(void);
    void update_stopping(void);
    void update_test(void);
    void blink_led(bool on);

    // Perturb-and-observe tuning of the commutation phase offset. Called once
    // per PID tick; steps _phase_deg every PHASE_PO_INTERVAL_MS. When
    // throttle_controlled is true the objective is electrical RPM (maximise
    // speed at the fixed throttle); otherwise it is negative duty (minimise
    // the drive the PID needs to hold the RPM setpoint).
    void update_phase_po(bool throttle_controlled);

    // TIM8 timing: _arr = half-period in timer ticks
    //   160 MHz / (2 × 20 kHz) = 4000 → TIM8->ARR programmed as _arr - 1
    uint32_t _arr      = 4000U;
    float    _throttle = 0.0f;
    bool     _enabled  = false;

    // Control mode
    DriveMode _mode = DriveMode::IDLE;

    // Startup sequencing — pulse/rest until hall toggles, then hand off to ISR
    uint32_t _kick_start_ms       = 0;   // overall startup entry time (0 = not entered)
    uint32_t _kick_phase_start_ms = 0;   // start of current pulse or rest segment
    bool     _kick_resting        = false;
    bool     _commute_active      = false;
    bool     _hall_at_kick_start  = false;
    uint16_t _kick_attempts       = 0;

    // Test mode
    bool _test_mode_active = false;
    bool _prev_deadman     = false;

    // SBUS RC input — channels latched from AP_RCProtocol on each new frame
    uint16_t _rc_channels[MAX_RCIN_CHANNELS] = {};
    uint8_t  _rc_num_channels = 0;
    uint32_t _rc_last_frame_ms = 0;
    bool     _rc_failsafe = false;

    // Speed: stored as electrical RPM (matches PID / startup constants).
    // get_rpm() converts to mechanical for external consumers.
    float _electrical_rpm = 0.0f;

    // Commutation phase offset (electrical degrees, signed) and its online
    // perturb-and-observe optimiser state. _phase_deg is mirrored into the
    // ISR-visible s_phase_deg whenever it changes; it persists across runs so
    // a learned offset is reused on the next spin-up.
    float    _phase_deg     = 0.0f;
    int8_t   _po_dir        = -1;     // search direction (start by advancing)
    bool     _po_primed     = false;  // baseline objective captured yet?
    float    _po_last_obj   = 0.0f;   // objective from the previous window
    float    _po_last_duty  = 0.0f;   // avg duty from the previous window
    uint32_t _po_window_ms  = 0;      // start of the current measure window
    float    _po_rpm_sum    = 0.0f;   // accumulators over the window
    float    _po_duty_sum   = 0.0f;
    uint16_t _po_samples    = 0;

    // PID (operates on electrical RPM)
    float    _pid_integral = 0.0f;
    float    _pid_last_err = 0.0f;
    uint32_t _pid_last_us  = 0;

    uint32_t _batt_print_ms = 0;

    // Peak tracking (reset each print interval). RPM peak is mechanical so
    // the printed value matches what a tachometer would read.
    float    _peak_mech_rpm        = 0.0f;
    float    _peak_throttle        = 0.0f;
    float    _peak_current         = 0.0f;
    float    _volt_at_peak_current = 0.0f;

    // Motor-lead voltage sense (PA4/PA7 read directly from ADC2). Latched at
    // the instant of peak mechanical RPM within each print interval, already
    // corrected for the 49.9k/2.2k divider.
    float    _vsense_a_at_peak     = 0.0f;
    float    _vsense_b_at_peak     = 0.0f;
};

#endif
