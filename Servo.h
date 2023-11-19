#ifndef SERVO_H_
#define SERVO_H_

#include "mbed.h"

// #include "PESBoardPinName.h"
#include "Motion.h"
#include "ThreadFlag.h"

/**
 * Servo class for smooth control of a servo motor.
 */
class Servo
{
public:
    explicit Servo(PinName pinName);
    virtual ~Servo();

    void calibratePulseMinMax(float pulse_min = 0.0f, float pulse_max = 1.0f);
    void setMotionProfileAcceleration(float acceleration = 1.0e6f); // 1.0e6f instead of infinity
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
    float m_pulse_min{0.0f};
    float m_pulse_max{1.0f};

    void threadTask();
    void enableDigitalOutput();
    void disableDigitalOutput();
    void sendThreadFlag();
    float constrainPulse(float pulse) const;

    // deleted copy constructor and copy assignment operator
    Servo(const Servo &) = delete;
    Servo &operator=(const Servo &) = delete;
};

#endif /* SERVO_H_ */
