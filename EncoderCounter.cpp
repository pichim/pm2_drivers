/*
 * EncoderCounter.cpp
 * Copyright (c) 2018, ZHAW
 * All rights reserved.
 */

#include "EncoderCounter.h"

using namespace std;

/**
 * Creates and initializes the driver to read the quadrature
 * encoder counter of the STM32 microcontroller.
 * @param a the input pin for the channel A.
 * @param b the input pin for the channel B.
 */
EncoderCounter::EncoderCounter(PinName a, PinName b)
{
    // check pins

    if ((a == PA_0) && (b == PA_1)) {

        // pinmap OK for TIM2 CH1 and CH2

        TIM = TIM2;

        // configure general purpose I/O registers

        GPIOA->MODER &= ~GPIO_MODER_MODER0;     // reset port A0
        GPIOA->MODER |= GPIO_MODER_MODER0_1;    // set alternate mode of port A0
        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;     // reset pull-up/pull-down on port A0
        GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;    // set input as pull-down
        GPIOA->AFR[0] &= ~(0xF << 4*0);         // reset alternate function of port A0
        GPIOA->AFR[0] |= 1 << 4*0;              // set alternate funtion 1 of port A0

        GPIOA->MODER &= ~GPIO_MODER_MODER1;     // reset port A1
        GPIOA->MODER |= GPIO_MODER_MODER1_1;    // set alternate mode of port A1
        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;     // reset pull-up/pull-down on port A1
        GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_1;    // set input as pull-down
        GPIOA->AFR[0] &= ~(0xF << 4*1);         // reset alternate function of port A1
        GPIOA->AFR[0] |= 1 << 4*1;              // set alternate funtion 1 of port A1

        // configure reset and clock control registers

        RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;  //reset TIM2 controller
        RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;

        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;     // TIM2 clock enable

    } else if ((a == PA_6) && (b == PC_7)) {

        // pinmap OK for TIM3 CH1 and CH2

        TIM = TIM3;

        // configure reset and clock control registers

        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;    // manually enable port C (port A enabled by mbed library)

        // configure general purpose I/O registers

        GPIOA->MODER &= ~GPIO_MODER_MODER6;     // reset port A6
        GPIOA->MODER |= GPIO_MODER_MODER6_1;    // set alternate mode of port A6
        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR6;     // reset pull-up/pull-down on port A6
        GPIOA->PUPDR |= GPIO_PUPDR_PUPDR6_1;    // set input as pull-down
        GPIOA->AFR[0] &= ~(0xF << 4*6);         // reset alternate function of port A6
        GPIOA->AFR[0] |= 2 << 4*6;              // set alternate funtion 2 of port A6

        GPIOC->MODER &= ~GPIO_MODER_MODER7;     // reset port C7
        GPIOC->MODER |= GPIO_MODER_MODER7_1;    // set alternate mode of port C7
        GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR7;     // reset pull-up/pull-down on port C7
        GPIOC->PUPDR |= GPIO_PUPDR_PUPDR7_1;    // set input as pull-down
        GPIOC->AFR[0] &= ~0xF0000000;           // reset alternate function of port C7
        GPIOC->AFR[0] |= 2 << 4*7;              // set alternate funtion 2 of port C7

        // configure reset and clock control registers

        RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;  //reset TIM3 controller
        RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;

        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;     // TIM3 clock enable

    } else if ((a == PB_6) && (b == PB_7)) {

        // pinmap OK for TIM4 CH1 and CH2

        TIM = TIM4;

        // configure reset and clock control registers

        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    // manually enable port B (port A enabled by mbed library)

        // configure general purpose I/O registers

        GPIOB->MODER &= ~GPIO_MODER_MODER6;     // reset port B6
        GPIOB->MODER |= GPIO_MODER_MODER6_1;    // set alternate mode of port B6
        GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR6;     // reset pull-up/pull-down on port B6
        GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_1;    // set input as pull-down
        GPIOB->AFR[0] &= ~(0xF << 4*6);         // reset alternate function of port B6
        GPIOB->AFR[0] |= 2 << 4*6;              // set alternate funtion 2 of port B6

        GPIOB->MODER &= ~GPIO_MODER_MODER7;     // reset port B7
        GPIOB->MODER |= GPIO_MODER_MODER7_1;    // set alternate mode of port B7
        GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR7;     // reset pull-up/pull-down on port B7
        GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_1;    // set input as pull-down
        GPIOB->AFR[0] &= ~0xF0000000;           // reset alternate function of port B7
        GPIOB->AFR[0] |= 2 << 4*7;              // set alternate funtion 2 of port B7

        // configure reset and clock control registers

        RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST;  //reset TIM4 controller
        RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM4RST;

        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;     // TIM4 clock enable

    } else {

        printf("pinmap not found for peripheral\n");
    }

    // configure general purpose timer 3 or 4

    TIM->CR1 = 0x0000;          // counter disable
    TIM->CR2 = 0x0000;          // reset master mode selection
    TIM->SMCR = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0; // counting on both TI1 & TI2 edges
    TIM->CCMR1 = TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_0;
    TIM->CCMR2 = 0x0000;        // reset capture mode register 2
    TIM->CCER = TIM_CCER_CC2E | TIM_CCER_CC1E;
    TIM->CNT = 0x0000;          // reset counter value
    TIM->ARR = 0xFFFF;          // auto reload register
    TIM->CR1 = TIM_CR1_CEN;     // counter enable
}

EncoderCounter::~EncoderCounter() {}

/**
 * Resets the counter value to zero.
 */
void EncoderCounter::reset()
{
    TIM->CNT = 0x0000;
}

/**
 * Resets the counter value to a given offset value.
 * @param offset the offset value to reset the counter to.
 */
void EncoderCounter::reset(int16_t offset)
{
    TIM->CNT = -offset;
}

/**
 * Reads the quadrature encoder counter value.
 * @return the quadrature encoder counter as a signed 16-bit integer value.
 */
int16_t EncoderCounter::read()
{
    return static_cast<int16_t>(-TIM->CNT);
}

/**
 * The empty operator is a shorthand notation of the <code>read()</code> method.
 */
EncoderCounter::operator int16_t()
{
    return read();
}



