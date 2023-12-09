#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

#include "EncoderCounter.h"
#include "FastPWM/FastPWM.h"
#include "LowpassFilter.h"
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

    void setVelocityCntrlGain(float kp = 5.0f);
    void setRotationCntrlGain(float p = 10.0f);

    void setMaxVelocity(float velocity);
    void setMaxAcceleration(float acceleration = 1.0e6f); // 0.5f

private:
    static constexpr int64_t PERIOD_MUS = 200;
    static constexpr float TS = 1.0e-6f * static_cast<float>(PERIOD_MUS);
    // static constexpr float LOWPASS_FILTER_FREQUENCY_RAD = 120.0f;
    static constexpr float PWM_MIN = 0.01f;
    static constexpr float PWM_MAX = 0.99f;
    static constexpr int PWM_PERIOD_MUS = 50;

    FastPWM m_FastPWM;
    EncoderCounter m_EncoderCounter;
    // LowpassFilter m_LowpassFilter;
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
    float m_kn;
    float m_voltage_max;
    float m_velocity_max;

    // controller parameters
    float m_kp{0.0f};
    float m_ki{1.0f};
    float m_p{0.0f};

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