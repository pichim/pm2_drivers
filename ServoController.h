#ifndef SERVO_CONTROLLER_H_
#define SERVO_CONTROLLER_H_

#include <mbed.h>

#include "Servo.h"
#include "Motion.h"
#include "ThreadFlag.h"

/**
 * Servo class.
 */
class ServoController
{

public:
    ServoController(PinName pinName);
    virtual ~ServoController();

    void setNormalisedPulseWidth(float pulse);
    void enable(float pulse);
    void enable();
    void disable();
    bool isEnabled();

private:
    static const float TS;

    float m_pulse;

    Servo m_Servo;
    Motion m_Motion;
    ThreadFlag m_ThreadFlag;
    Thread m_Thread;
    Ticker m_Ticker;

    void run();
    void sendThreadFlag();
};

#endif /* SERVO_CONTROLLER_H_ */