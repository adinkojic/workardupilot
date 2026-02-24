#pragma once

#if AP_PERIPH_SINGLE_PHASE_BLDC

class SinglePhaseBLDC {
public:
    SinglePhaseBLDC(); //idk if a constructor is necessary or practical
    friend class AP_Periph_FW;

    // main update function
    void update(void);

private:
};
#endif