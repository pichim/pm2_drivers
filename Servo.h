#ifndef SERVO_H_
#define SERVO_H_

#include "mbed.h"

/**
 * Servo class.
 */
class Servo
{
public:
    explicit Servo(PinName pinName);
    virtual ~Servo();

    static constexpr uint16_t PERIOD_MUS = 20000;

    void setNormalisedPulseWidth(float pulse = 0.0f);
    void enable(float pulse = 0.0f);
    void disable();
    bool isEnabled() const;
    float constrainPulse(float pulse) const;

private:
    static constexpr float INPUT_MIN = 0.01f;
    static constexpr float INPUT_MAX = 0.99f;

    DigitalOut m_DigitalOut;
    Ticker m_Ticker;
    Timeout m_Timeout;

    bool m_servo_enabled;
    uint16_t m_pulse_mus;

    void startPulse();
    void endPulse();
};

#endif /* SERVO_H_ */
