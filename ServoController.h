#ifndef SERVO_CONTROLLER_H_
#define SERVO_CONTROLLER_H_

#include "mbed.h"

#include "Servo.h"
#include "Motion.h"
#include "ThreadFlag.h"

/**
 * ServoController class with motion control.
 */
class ServoController
{
public:
    explicit ServoController(PinName pinName);
    virtual ~ServoController();

    void setNormalisedPulseWidth(float pulse = 0.0f);
    void enable(float pulse = 0.0f);
    void disable();
    bool isEnabled() const;

private:
    static constexpr float PROFILE_VELOCITY = 1.0e6f; // 1.0e6f instead of inf
    static constexpr float PROFILE_ACCELERATION = 0.2f;
    static constexpr float TS = 1.0e-6f * static_cast<float>(Servo::PERIOD_MUS);

    Servo m_Servo;
    Motion m_Motion;
    ThreadFlag m_ThreadFlag;
    Thread m_Thread;
    Ticker m_Ticker;

    float m_pulse;

    void run();
    void sendThreadFlag();
};

#endif /* SERVO_CONTROLLER_H_ */
