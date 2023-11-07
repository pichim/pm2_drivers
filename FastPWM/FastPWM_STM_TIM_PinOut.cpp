#include "mbed.h"

#if defined (TARGET_NUCLEO_F030R8) || (TARGET_DISCO_F051R8)
__IO uint32_t* getChannel(TIM_TypeDef* pwm, PinName pin) {
    switch (pin) {
        // Channels 1
        case PA_4: case PA_6: case PB_1: case PB_4: case PB_8: case PB_9: case PB_14: case PC_6: case PB_6: case PB_7:
            return &pwm->CCR1;
            
        // Channels 2
        case PA_7: case PB_5: case PC_7:
            return &pwm->CCR2;
            
        // Channels 3
        case PB_0: case PC_8:
            return &pwm->CCR3;
            
        // Channels 4
        case PC_9:
            return &pwm->CCR4;
    }        
    return NULL;
}
#endif

#if defined (TARGET_NUCLEO_F103RB) || (TARGET_DISCO_F100RB)
__IO uint32_t* getChannel(TIM_TypeDef* pwm, PinName pin) {
    switch (pin) {
        // Channels 1 : PWMx/1
        case PA_6: case PA_8: case PA_15: case PB_4: case PC_6: case PB_13:
            return &pwm->CCR1;
        
        // Channels 2 : PWMx/2
        case PA_1: case PA_7: case PA_9: case PB_3: case PB_5: case PC_7: case PB_14:
            return &pwm->CCR2;
            
        // Channels 3 : PWMx/3
        case PA_2: case PA_10: case PB_0: case PB_10: case PC_8: case PB_15:
            return &pwm->CCR3;
 
        // Channels 4 : PWMx/4
        case PA_3: case PA_11: case PB_1: case PB_11: case PC_9: 
            return &pwm->CCR4;
    }
    return NULL;
}
#endif

#if defined TARGET_NUCLEO_F072RB
__IO uint32_t* getChannel(TIM_TypeDef* pwm, PinName pin) {
    switch (pin) {
        // Channels 1 : PWMx/1
        case PA_2: case PA_6: case PA_4: case PA_7: case PA_8: case PB_1: case PB_4: case PB_8: case PB_9: case PB_14: case PC_6:
        // Channels 1N : PWMx/1N
        case PA_1: case PB_6: case PB_7: case PB_13:
            return &pwm->CCR1;
        
        // Channels 2 : PWMx/2
        case PA_3: case PA_9: case PB_5: case PC_7: case PB_15:
            return &pwm->CCR2;
            
        // Channels 3 : PWMx/3
        case PA_10: case PB_0: case PC_8: 
            return &pwm->CCR3;
 
        // Channels 4 : PWMx/4
        case PA_11: case PC_9: 
            return &pwm->CCR4;
    }
    return NULL;
}
#endif



#if defined (TARGET_NUCLEO_L152RE)
__IO uint32_t* getChannel(TIM_TypeDef* pwm, PinName pin) {
    switch (pin) {
        // Channels 1 : PWMx/1
        case PA_6:  case PB_4: case PB_12: case PB_13: case PC_6:
            return &pwm->CCR1;
        
        // Channels 2 : PWMx/2
        case PA_1: case PA_7: case PB_3: case PB_5: case PB_14: case PB_7: case PC_7:
            return &pwm->CCR2;
            
        // Channels 3 : PWMx/3
        case PA_2: case PB_0: case PB_8: case PB_10: case PC_8:
            return &pwm->CCR3;
 
        // Channels 4 : PWMx/4
        case PA_3: case PB_1:case PB_9: case PB_11: case PC_9: 
            return &pwm->CCR4;
        default:
            /* NOP */
            break;
    }
    return NULL;
}
#endif

 