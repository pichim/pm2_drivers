#ifndef SERVO_H_
#define SERVO_H_

#include "mbed.h"

/**
 * Servo controller for smooth movements.
 */
class Servo
{

public:
    Servo(PinName pinName);
    virtual ~Servo();

    void setNormalisedPulseWidth(float pulse);
    void enable(float pulse);
    void enable();
    void disable();
    bool isEnabled();
    float constrainPulse(float pulse);

private:
    static const float INPUT_MIN;
    static const float INPUT_MAX;
    static const uint16_t PERIOD_MUS;

    bool m_servo_enabled;
    uint16_t m_pulse_mus, m_period_mus;

    DigitalOut m_DigitalOut;
    Ticker m_Ticker;
    Timeout m_Timeout;

    void startPulse();
    void endPulse();
};

#endif /* SERVO_H_ */