#ifndef SERVO_SMOOTH_H_
#define SERVO_SMOOTH_H_

#include "mbed.h"

#include "Motion.h"
#include "ThreadFlag.h"

/**
 * ServoSmooth class for smooth control of a servo motor.
 */
class ServoSmooth
{
public:
    explicit ServoSmooth(PinName pinName);
    virtual ~ServoSmooth();

    void setProfileAcceleration(float acceleration = 0.1f);
    void setNormalisedPulseWidth(float pulse = 0.0f);
    void enable(float pulse = 0.0f);
    void disable();
    bool isEnabled() const;

private:
    static constexpr uint16_t PERIOD_MUS = 20000;
    static constexpr float INPUT_MIN = 0.01f;
    static constexpr float INPUT_MAX = 0.99f;

    DigitalOut m_DigitalOut;
    Motion m_Motion;
    Timeout m_Timeout;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    bool m_enabled{false};
    float m_pulse{0.0f};

    void threadTask();
    void enableDigitalOutput();
    void disableDigitalOutput();
    void sendThreadFlag();
    float constrainPulse(float pulse) const;

    // Deleted copy constructor and copy assignment operator
    ServoSmooth(const ServoSmooth &) = delete;
    ServoSmooth &operator=(const ServoSmooth &) = delete;
};

#endif /* SERVO_SMOOTH_H_ */
