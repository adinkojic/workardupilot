#pragma once

#if AP_PERIPH_SINGLE_PHASE_BLDC

class SinglePhaseBLDC {
public:
    friend class AP_Periph_FW;

    // main update function
    void update(void);
    void init(void);

private:
    void blink_led(uint32_t now);
    void clear_fault(void);
    //AP_HAL::OwnPtr<AP_HAL::SPIDevice> _spi;
};
#endif