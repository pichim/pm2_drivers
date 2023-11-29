#ifndef SERVO_H_
#define SERVO_H_

#include "mbed.h"

#include "Motion.h"
#include "ThreadFlag.h"

/**
 * @brief Class for smooth control of a servo motor.
 *
 * This class provides functionalities to control servo motors with
 * features like calibration, setting motion profiles, and enabling/disabling the servo.
 */
class Servo
{
public:
    /**
     * @brief Construct a new Servo object.
     * 
     * @param pinName The pin name to which the servo is connected.
     */
    explicit Servo(PinName pinName);

    /**
     * @brief Destroy the Servo object.
     */
    virtual ~Servo();

    /**
     * @brief Calibrate the minimum and maximum pulse widths for the servo.
     * 
     * @param pulse_min The minimum pulse width.
     * @param pulse_max The maximum pulse width.
     */
    void calibratePulseMinMax(float pulse_min = 0.0f, float pulse_max = 1.0f);

    /**
     * @brief Set the motion profile acceleration.
     * 
     * @param acceleration The acceleration value for the motion profile.
     */
    void setMotionProfileAcceleration(float acceleration = 1.0e6f); // 1.0e6f instead of infinity

    /**
     * @brief Set the normalised pulse width.
     * 
     * @param pulse The pulse width to be set, normalised.
     */
    void setNormalisedPulseWidth(float pulse = 0.0f);

    /**
     * @brief Enable the servo with a specific pulse width.
     * 
     * @param pulse The pulse width to set when enabling.
     */
    void enable(float pulse = 0.0f);

    /**
     * @brief Disable the servo.
     */
    void disable();

    /**
     * @brief Check if the servo is enabled.
     * 
     * @return true If the servo is enabled.
     * @return false If the servo is disabled.
     */
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

    float calculateNormalisedPulseWidth(float pulse);
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
