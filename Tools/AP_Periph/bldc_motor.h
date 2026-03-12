#pragma once

#if AP_PERIPH_SINGLE_PHASE_BLDC

class SinglePhaseBLDC {
public:
    friend class AP_Periph_FW;

    void update(void);
    void init(void);

    // Set motor speed: 0.0 = stopped, 1.0 = full speed
    void set_throttle(float throttle);

    // Estimated RPM from hall sensor edge timing
    uint32_t get_rpm(void) const;

    // Number of pole pairs on the motor (default 1)
    uint8_t pole_pairs = 4;

private:
    void blink_led(uint32_t now);
    void clear_fault(void);
};

#endif