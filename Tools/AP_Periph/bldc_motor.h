#pragma once

#if AP_PERIPH_SINGLE_PHASE_BLDC

class SinglePhaseBLDC {
public:
    friend class AP_Periph_FW;

    // main update function
    void update(void);
    void init(void);

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _spi;
    bool clear_fault(void);
};
#endif