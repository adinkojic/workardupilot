#pragma once

#if AP_PERIPH_SINGLE_PHASE_BLDC

// --- Tunable parameters ---------------------------------------------------
#define STARTUP_KICK_THROTTLE    0.6f     // duty cycle for initial kick pulse
#define STARTUP_KICK_MS          50U      // kick pulse duration (ms)
#define STARTUP_COMMUTE_THROTTLE 0.5f     // duty cycle during startup commutation
#define STARTUP_RPM_THRESHOLD    500.0f   // electrical RPM to enter PID mode

#define TARGET_RPM               1000.0f  // PID setpoint (electrical RPM)
#define PID_KP                   0.001f
#define PID_KI                   0.0001f
#define PID_KD                   0.0f
#define PID_INTEGRAL_MAX         5.0f     // anti-windup clamp

#define POLE_PAIRS               4U       // mechanical RPM = electrical RPM / POLE_PAIRS
#define RPM_TIMEOUT_US           500000U  // no hall edge for this long → RPM = 0
#define STOP_RAMP_MS             500U     // ramp-down duration when deadman released

#define TEST_THROTTLE            1.0f     // duty cycle used in H-bridge test mode
// --------------------------------------------------------------------------

class SinglePhaseBLDC {
public:
    friend class AP_Periph_FW;

    void  update(void);
    void  init(void);

    void  set_enabled(bool en);
    void  set_throttle(float throttle);
    void  flip_direction(void);
    void  set_frequency(uint32_t hz);
    float get_rpm(void) const { return _actual_rpm; }

    // Enable H-bridge test mode: hold button = bridge on, release = off,
    // each press flips direction. Overrides normal STARTUP/PID flow.
    void  set_test_mode(bool en) { _test_mode_active = en; }

private:
    enum class DriveMode : uint8_t { IDLE, STARTUP, PID, STOPPING, TEST };

    void ccr_apply(void);
    void compute_rpm(void);
    void update_startup(void);
    void update_pid(void);
    void update_stopping(void);
    void update_test(void);
    void blink_led(bool on);

    // TIM8 state (4000 = 160 MHz / (2 * 20 kHz))
    uint32_t _arr      = 4000U;
    float    _throttle = 0.0f;
    bool     _enabled  = false;

    // Control mode
    DriveMode _mode = DriveMode::IDLE;

    // Startup sequencing
    uint32_t _kick_start_ms = 0;
    bool     _kick_done     = false;

    // Stopping ramp
    uint32_t _stop_start_ms       = 0;
    float    _stop_start_throttle = 0.0f;

    // Test mode
    bool _test_mode_active = false;
    bool _prev_deadman     = false;

    // RPM (computed in main loop from ISR-maintained timing)
    float _actual_rpm = 0.0f;

    // PID
    float    _pid_integral = 0.0f;
    float    _pid_last_err = 0.0f;
    uint32_t _pid_last_us  = 0;

    uint32_t _batt_print_ms = 0;
};

#endif
