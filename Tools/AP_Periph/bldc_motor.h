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
// Handoff speed to PID (pulse-width) mode. Keep this high enough that the
// pulse drive is safe when it takes over: pulses apply full rail voltage with
// no chop, so back-EMF has to be doing the current limiting. At 50k eRPM the
// half period is 600 µs and the widest ON window (0.8 × 300 µs = 240 µs) sits
// just under PULSE_MAX_ON_US — below this speed the cap would strangle the
// drive anyway, and the winding current of an uncapped pulse would only be
// limited by the winding resistance.
#define STARTUP_RPM_THRESHOLD    50000.0f  // electrical RPM to enter PID mode

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
#define SBUS_THROTTLE_OUT_MAX    0.98f
#define SBUS_TIMEOUT_MS          200U      // no frame for this long → off

// --- Simulation mode (SBUS channel 5 high) ---------------------------------
// With channel 5 above the threshold the real hall sensor is disconnected from
// the control path and replaced by a simulated rotor, while the H-bridge stays
// fully live — so the whole drive stack (startup kicks, hall-commutated
// spin-up, pulse scheduler, PID) can be exercised and scoped with no motor
// attached. The rotor model matches the observed motor: 4 pole pairs → 8 hall
// edges (45° each) per mechanical rev, ~69600 RPM at ~50% throttle on 33 V,
// ~200 ms spin-up, ~500 ms coast-down. If a large battery current is seen
// while simulating (a real motor was left connected and just started), the
// drive is stopped and locked out for SIM_FAULT_DISABLE_MS with the LED
// flashing rapidly. All of this is simulation-only — nothing changes when
// channel 5 is low.
#define SBUS_SIM_CHANNEL         5U        // 1-based channel enabling simulation
#define SBUS_SIM_THRESHOLD_US    1700U     // channel above this = simulate
// Steady-state speed scales linearly with throttle: 69600 mech RPM observed
// at ~50% throttle on 33 V → 139200 RPM per unit throttle.
#define SIM_RPM_PER_THROTTLE     139200.0f
#define SIM_ACCEL_TAU_S          0.10f     // spin-up time constant (~200 ms to 60k)
#define SIM_DECEL_RPM_PER_S      139200.0f // coast-down (69600 → 0 in ~500 ms)
// Above this speed simulated hall edges are chained by the TIM5 CC3 compare
// ISR (at 69600 RPM edges come at 9.3 kHz — far beyond the 1 kHz main loop);
// below it the main loop emits edges from the integrated rotor angle so slow
// motion (the startup kicks) advances the hall realistically.
#define SIM_ISR_HANDOFF_RPM      3000.0f
#define SIM_FAULT_CURRENT_A      2.0f      // battery amps while simulating ⇒ motor connected
#define SIM_FAULT_DISABLE_MS     10000U    // lockout after a simulation current fault
#define SIM_HALL_FAKE_PIN        0xFFU     // pin id marking simulated hall_irq calls
// ----------------------------------------------------------------------------

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
// every consumer (startup kick, immediate commutation, and the pulse-width
// scheduler) routes through hall_dir(), so nothing else needs touching.
#define HALL_POLARITY            (+1)

// --- Pulse-width drive (post-startup) -------------------------------------
// Once spun up the driver stops chopping the current. The hall handler keeps a
// rolling average of the half period (one hall pulse width) over the last
// SPEED_AVG_CYCLES edges and, on each edge, schedules a single H-bridge pulse
// via two TIM5 one-shots:
//   - it turns ON  1/4 pulse width (= 1/8 electrical cycle) after the edge,
//     plus a fixed PULSE_DELAY_OFFSET_US,
//   - it stays ON for (throttle × 1/2 pulse width),
// so throttle sweeps the drive window across the middle 50% of each hall
// pulse. The bridge is fully active (no PWM chop) for the window and disabled
// (MOE off, winding floating) the rest of the time. Both edges arm a fresh
// single-shot, so if edges stop the bridge simply stays disabled; after
// PULSE_STOP_TIMEOUT_MS with no edge the driver returns to IDLE.
#define SPEED_AVG_CYCLES         8U        // half-periods in the rolling speed average
#define PULSE_DELAY_OFFSET_US    0         // extra hall-edge → H-bridge-ON delay
// Hard ceiling on a single unchopped ON window. The pulse drive applies full
// rail voltage, so on a slow rotor (little back-EMF) the winding current is
// limited only by resistance and rises with L/R towards V_bus/R — long pulses
// are what kill MOSFETs. 250 µs is generous next to the ~24 µs ON window at
// TARGET_RPM but bounds the damage from a bad speed estimate or an early
// handoff. Size it against the winding L/R and the FET pulsed-current rating.
#define PULSE_MAX_ON_US          250U
#define PULSE_STOP_TIMEOUT_MS    1000U     // no hall edge this long → back to IDLE
#define PULSE_STOP_TIMEOUT_US    (PULSE_STOP_TIMEOUT_MS * 1000U)
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

    // Simulated hall edge generator, called from the TIM5 CC3 compare ISR
    // (a free function, hence public static). Toggles the simulated hall
    // level, feeds it to hall_irq() and chains the next edge.
    static void sim_hall_edge_isr(void);

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

    // Hall level as seen by the control logic — the simulated level while
    // simulation is active, the real pin otherwise. The startup state machine
    // reads the hall through this so simulation only swaps the data source.
    bool read_hall(void) const;

    // Simulation engine (main loop, 1 kHz): handles entry/exit on the SBUS
    // request, integrates the rotor model and emits/chains fake hall edges.
    void update_simulation(bool requested);

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

    // Simulation mode — rotor model state, main-loop only (the ISR-shared
    // pieces are file statics in bldc_motor.cpp)
    bool     _sim_active     = false;
    float    _sim_mech_rpm   = 0.0f;   // simulated mechanical RPM
    float    _sim_angle_deg  = 0.0f;   // simulated rotor angle [0, 360)
    float    _sim_edge_accum = 0.0f;   // degrees since the last emitted edge
    uint32_t _sim_last_us    = 0;      // model integration timestamp
    uint32_t _sim_fault_ms   = 0;      // 0 = no fault; else millis() at trigger

    // SBUS RC input — channels latched from AP_RCProtocol on each new frame
    uint16_t _rc_channels[MAX_RCIN_CHANNELS] = {};
    uint8_t  _rc_num_channels = 0;
    uint32_t _rc_last_frame_ms = 0;
    bool     _rc_failsafe = false;

    // Speed: stored as electrical RPM (matches PID / startup constants).
    // get_rpm() converts to mechanical for external consumers.
    float _electrical_rpm = 0.0f;

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
