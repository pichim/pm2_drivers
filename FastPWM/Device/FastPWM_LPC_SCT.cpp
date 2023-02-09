//For targets which use the SCT
#if defined(TARGET_LPC81X) || defined(TARGET_LPC82X)

#ifdef TARGET_LPC82X
#define CTRL_U CTRL
#endif

#include "FastPWM.h"

void FastPWM::initFastPWM( void ) {
    //Mbed uses the timer as a single unified 32-bit timer, who are we to argue with this, and it is easier
    bits = 32;
    
    #ifdef TARGET_LPC82X
    //The mbed lib uses the PWM peripheral slightly different, which is irritating. This sets it bck to the LPC81X
        _pwm.pwm->EVENT[_pwm.pwm_ch + 1].CTRL  = (1 << 12) | (_pwm.pwm_ch + 1);     // Event_n on Match_n 
        _pwm.pwm->EVENT[_pwm.pwm_ch + 1].STATE = 0xFFFFFFFF;                        // All states  
        _pwm.pwm->OUT[_pwm.pwm_ch].SET = (1 << 0);                                  // All PWM channels are SET on Event_0 
        _pwm.pwm->OUT[_pwm.pwm_ch].CLR = (1 << (_pwm.pwm_ch + 1));                  // PWM ch is CLRed on Event_(ch+1) 
    #endif
    
    //With 32-bit we fix prescaler to 1
    _pwm.pwm->CTRL_U |= (1 << 2) | (1 << 3);
    _pwm.pwm->CTRL_U &= ~(0x7F << 5);
    _pwm.pwm->CTRL_U &= ~(1 << 2);

}

void FastPWM::pulsewidth_ticks( uint32_t ticks ) {
    #ifdef TARGET_LPC81X
    _pwm.pwm->MATCHREL[_pwm.pwm_ch + 1].U = ticks;
    #else
    _pwm.pwm->MATCHREL[_pwm.pwm_ch + 1] = ticks;
    #endif
}

void FastPWM::period_ticks( uint32_t ticks ) {
    #ifdef TARGET_LPC81X
    _pwm.pwm->MATCHREL[0].U = ticks;
    #else
    _pwm.pwm->MATCHREL[0] = ticks;
    #endif
}

uint32_t FastPWM::getPeriod( void ) {
    #ifdef TARGET_LPC81X
    return _pwm.pwm->MATCHREL[0].U;
    #else
    return _pwm.pwm->MATCHREL[0];
    #endif
}

//Maybe implemented later, but needing to change the prescaler for a 32-bit
//timer used in PWM mode is kinda unlikely.
//If you really need to do it, rejoice, you can make it run so slow a period is over 40,000 year
uint32_t FastPWM::setPrescaler(uint32_t reqScale) {
    //Disable dynamic prescaling
    dynamicPrescaler = false;
    
    return 1;
}

#endif