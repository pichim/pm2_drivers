//This should (hopefully) work on all STM targets which use TIM timers for PWM

#ifdef TARGET_STM

#include "FastPWM.h"

typedef __IO uint32_t* CHANNEL_P_T;

typedef struct  {
    CHANNEL_P_T channel;
    uint32_t clk_prescaler;
} fastpwm_struct;

#define PWM_CHANNEL         ((((fastpwm_struct*)fast_obj)->channel)) 
#define PWM_CLK_PRESCALER   ((((fastpwm_struct*)fast_obj)->clk_prescaler))
#define PWM_TIMER           ((TIM_TypeDef*)_pwm.pwm)

#if defined(TARGET_STM32F0) || defined (TARGET_STM32F1) || defined (TARGET_STM32L1)
extern __IO uint32_t* getChannel(TIM_TypeDef* pwm, PinName pin);
#endif

void FastPWM::initFastPWM( void ) {
    fast_obj = malloc(sizeof(fastpwm_struct));
    #if defined(TARGET_STM32F0) || defined (TARGET_STM32F1) || defined (TARGET_STM32L1)
    PWM_CHANNEL = getChannel(PWM_TIMER, _pwm.pin);
    #else
    PWM_CHANNEL = (&PWM_TIMER->CCR1 + _pwm.channel - 1); 
    #endif
    
    // Depending on the timer and the internal bus it is connected to, each STM timer
    // can have a fixed prescaler from the clock, especially the faster devices.
    // In order not to have to hardcode this in, we use knowledge that mbed lib sets
    // default period to 20ms to reverse engineer the prescaler from this. 
    uint32_t current_hz = SystemCoreClock / (PWM_TIMER->PSC + 1) / (PWM_TIMER->ARR+1);
    PWM_CLK_PRESCALER = (current_hz + 1) / 50;  //50Hz is magic number it should be, +1 is to handle possible rounding errors in mbed setup
    
    //Sanity check in case a target does something different
    if ( (PWM_CLK_PRESCALER == 0 ) || (PWM_CLK_PRESCALER > 16)) {
        PWM_CLK_PRESCALER = 1;
    }
    
    //Enable PWM period syncing for glitch free result
    PWM_TIMER->CR1 |= TIM_CR1_ARPE;
    
    bits = 16;
}

void FastPWM::pulsewidth_ticks( uint32_t ticks ) {
    *PWM_CHANNEL = ticks;    
}

void FastPWM::period_ticks( uint32_t ticks ) {
    PWM_TIMER->ARR = ticks - 1;
}

uint32_t FastPWM::getPeriod( void ) {
    return PWM_TIMER->ARR + 1;
}

uint32_t FastPWM::setPrescaler(uint32_t reqScale) {
    if (reqScale == 0) {
        //Return prescaler
        return (PWM_TIMER->PSC + 1) * PWM_CLK_PRESCALER;
    }
    if (reqScale > (uint32_t)(PWM_CLK_PRESCALER<<16)) {
        reqScale = PWM_CLK_PRESCALER<<16;
    }
    //Else set prescaler, we have to substract one from reqScale since a 0 in PCVAL is prescaler of 1
    //Take into account PWM_CLK_PRESCALER, we need to make sure reqScale is always rounded up
    PWM_TIMER->PSC = (reqScale + PWM_CLK_PRESCALER - 1)/PWM_CLK_PRESCALER - 1;

    return setPrescaler(0);
}

#endif