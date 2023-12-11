#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

#include "EncoderCounter.h"
#include "FastPWM/FastPWM.h"
#include "Motion.h"
#include "ThreadFlag.h"
#include "PID_Cntrl.h"
#include "IIR_Filter.h"

class DCMotor
{
public:
    explicit DCMotor(PinName pin_pwm, PinName pin_enc_a, PinName pin_enc_b, float counts_per_turn, float kn, float voltage_max);

    virtual ~DCMotor();

    void setVelocity(float velocity);
    void setRotation(float rotation);

    float getRotationTarget() const;
    float getRotationSetpoint() const;
    float getRotation() const;
    float getVelocityTarget() const;
    float getVelocitySetpoint() const;
    float getVelocity() const;
    float getVoltage() const;
    float getPWM() const;

    void setVelocityCntrl(float kp = 3.5f, float ki = 3.5f / 0.02f, float kd = 0.12f);
    void setRotationCntrlGain(float p = 20.0f);

    void setMaxVelocity(float velocity);
    void setMaxAcceleration(float acceleration = 5.0f);

private:
    static constexpr int64_t PERIOD_MUS = 1000;
    static constexpr float TS = 1.0e-6f * static_cast<float>(PERIOD_MUS);
    static constexpr float PWM_MIN = 0.01f;
    static constexpr float PWM_MAX = 0.99f;
    static constexpr float ROTATION_ERROR_MAX = 5.0e-3f;

    FastPWM m_FastPWM;
    EncoderCounter m_EncoderCounter;
    Motion m_Motion;
    PID_Cntrl m_PID_Cntrl_velocity;
    IIR_Filter m_IIR_Filter_velocity;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    enum CntrlMode
    {
        Velocity,
        Rotation,
    };
    CntrlMode m_cntrlMode = CntrlMode::Velocity;

    // motor parameters
    float m_counts_per_turn;
    float m_voltage_max;
    float m_velocity_max;

    // rotation controller parameter
    float m_p;

    // signals
    short m_counts_previous;
    float m_rotation_initial;
    float m_rotation_target;
    float m_rotation_setpoint;
    float m_rotation;
    float m_velocity_target;
    float m_velocity_setpoint;
    float m_velocity;
    float m_voltage;
    float m_pwm;

    void threadTask();
    void sendThreadFlag();
};

#endif /* DC_MOTOR_H_ */