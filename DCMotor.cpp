#include <math.h>
#define M_PI 3.14159265358979323846264338327950288

#include "DCMotor.h"

DCMotor::DCMotor(PinName pin_pwm, PinName pin_enc_a, PinName pin_enc_b, float counts_per_turn, float kn, float voltage_max)
    : m_FastPWM(pin_pwm), m_EncoderCounter(pin_enc_a, pin_enc_b), m_Thread(osPriorityHigh, 4096)
{
    // motor parameters
    m_counts_per_turn = counts_per_turn;
    m_kn = kn / 60.0f;
    m_voltage_max = voltage_max;
    m_velocity_max = 0.8f * m_kn * voltage_max; // only allow 80% of theoretical maximum speed

    // default controller gains
    setVelocityCntrlGain();
    setRotationCntrlGain();

    // pid controller
    m_PID_Cntrl_velocity.setup(2.0f,
                               2.0f / 0.01f,
                               0.08f,
                               1.0f / (2.0f * M_PI * 10.0f),
                               1.0f / (2.0f * M_PI * 20.0f),
                               TS,
                               m_voltage_max * (2.0f * PWM_MIN - 1.0f),
                               m_voltage_max * (2.0f * PWM_MAX - 1.0f));

    // iir filter
    m_IIR_Filter_velocity.setup(2.0f * M_PI * 20.0f,
                                1.0f,
                                TS,
                                1.0f);

    // initialise pwm
    m_FastPWM.period_mus(PWM_PERIOD_MUS); // pwm period of 50 us
    m_FastPWM.write(0.5f);                // duty-cycle of 50% -> 0V

    // initialise control signals
    m_counts_previous = m_EncoderCounter.read();
    m_rotation_initial = static_cast<float>(m_counts_previous) / counts_per_turn;
    m_rotation_target = m_rotation_initial;
    m_rotation_setpoint = m_rotation_initial;
    m_rotation = m_rotation_initial;
    m_velocity_target = 0.0f;
    m_velocity_setpoint = 0.0f;
    m_velocity = 0.0f;
    m_voltage = 0.0f;
    m_pwm = 0.0f;

    // m_LowpassFilter.setPeriod(TS);
    // m_LowpassFilter.setFrequency(LOWPASS_FILTER_FREQUENCY_RAD);
    // m_LowpassFilter.reset(0.0f);

    m_Motion.setPosition(0.0f);
    m_Motion.setProfileVelocity(m_velocity_max);
    setMaxAcceleration();

    // start thread
    m_Thread.start(callback(this, &DCMotor::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &DCMotor::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

DCMotor::~DCMotor()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

void DCMotor::setVelocity(float velocity)
{
    m_cntrlMode = CntrlMode::Velocity;
    m_velocity_target = velocity;
}

void DCMotor::setRotation(float rotation)
{
    m_cntrlMode = CntrlMode::Rotation;
    m_rotation_target = m_rotation_initial + rotation;
}

float DCMotor::getRotationTarget() const
{
    return m_rotation_target;
}

float DCMotor::getRotationSetpoint() const
{
    return m_rotation_setpoint;
}

float DCMotor::getRotation() const
{
    return m_rotation - m_rotation_initial;
}

float DCMotor::getVelocityTarget() const
{
    return m_velocity_target;
}

float DCMotor::getVelocitySetpoint() const
{
    return m_velocity_setpoint;
}

float DCMotor::getVelocity() const
{
    return m_velocity;
}

float DCMotor::getVoltage() const
{
    return m_voltage;
}

float DCMotor::getPWM() const
{
    return m_pwm;
}

void DCMotor::setVelocityCntrlGain(float kp)
{
    m_kp = kp;
}

void DCMotor::setRotationCntrlGain(float p)
{
    m_p = p;
}

void DCMotor::setMaxVelocity(float velocity)
{
    velocity = (velocity > m_velocity_max) ? m_velocity_max : velocity;
    m_Motion.setProfileVelocity(velocity);
}

void DCMotor::setMaxAcceleration(float acceleration)
{
    m_Motion.setProfileAcceleration(acceleration);
    m_Motion.setProfileDeceleration(acceleration);
}

void DCMotor::threadTask()
{
    // float voltage_min_ = m_voltage_max * (2.0f * PWM_MIN - 1.0f);
    // float voltage_max_ = m_voltage_max * (2.0f * PWM_MAX - 1.0f);
    // printf("m_counts_per_turn: %f\n", m_counts_per_turn);
    // printf("m_kn: %f\n", m_kn);
    // printf("m_voltage_max: %f\n", m_voltage_max);
    // printf("m_velocity_max: %f\n", m_velocity_max);
    // printf("voltage_min: %f\n", voltage_min_);
    // printf("voltage_max: %f\n", voltage_max_);
    // printf("PWM_MIN_: %f\n", 0.5f + 0.5f * voltage_min_ / m_voltage_max);
    // printf("PWM_MAX_: %f\n", 0.5f + 0.5f * voltage_max_ / m_voltage_max);

    while (true)
    {
        ThisThread::flags_wait_any(m_ThreadFlag);

        // update signals
        const short counts = m_EncoderCounter.read();
        const float rotation_increment = static_cast<float>(counts - m_counts_previous) / m_counts_per_turn;
        m_counts_previous = counts;

        m_rotation = m_rotation + rotation_increment;
        // m_velocity = m_LowpassFilter.filter(rotation_increment / TS);
        m_velocity = m_IIR_Filter_velocity.filter(rotation_increment / TS);

        float velocity_setpoint = 0.0f;
        switch (m_cntrlMode) {
            case CntrlMode::Rotation:
                m_Motion.incrementToPosition(m_rotation_target, TS);
                m_rotation_setpoint = m_Motion.getPosition();
                // m_rotation_setpoint = m_rotation_target;
                // update p-position controller
                velocity_setpoint = m_p * (m_rotation_setpoint - m_rotation);
                break;

            case CntrlMode::Velocity:
                m_Motion.incrementToVelocity(m_velocity_target, TS);
                velocity_setpoint = m_Motion.getVelocity();
                // velocity_setpoint = m_velocity_target;
                break;

            default:
                break;
        }

        // constrain velocity to (-m_velocity_max, m_velocity_max)
        velocity_setpoint = (velocity_setpoint > m_velocity_max) ? m_velocity_max : (velocity_setpoint < -m_velocity_max) ? -m_velocity_max : velocity_setpoint;

        // // // update p-controller with feedforward
        // // float voltage = m_kp * (velocity_setpoint - m_velocity) + velocity_setpoint / m_kn;
        // // static float voltage_min = m_voltage_max * (2.0f * PWM_MIN - 1.0f);
        // // static float voltage_max = m_voltage_max * (2.0f * PWM_MAX - 1.0f);
        // // voltage = (voltage > voltage_max) ? voltage_max : (voltage < voltage_min) ? voltage_min : voltage;
        // static float voltage_min = m_voltage_max * (2.0f * PWM_MIN - 1.0f);
        // static float voltage_max = m_voltage_max * (2.0f * PWM_MAX - 1.0f);
        // static float voltage_previous = 0.0f;
        // float voltage = voltage_previous + m_ki * (velocity_setpoint - rotation_increment / TS) * TS;
        // voltage = (voltage > voltage_max) ? voltage_max : (voltage < voltage_min) ? voltage_min : voltage;
        // voltage_previous = voltage;
        // voltage += m_kp * (velocity_setpoint - m_velocity) + 0.0f * velocity_setpoint / m_kn;
        // voltage = (voltage > voltage_max) ? voltage_max : (voltage < voltage_min) ? voltage_min : voltage;

        // const float voltage = m_PID_Cntrl_velocity.update(velocity_setpoint, m_velocity, rotation_increment / TS, m_velocity);
        const float voltage = m_PID_Cntrl_velocity.update(velocity_setpoint - rotation_increment / TS, m_velocity);

        // calculate and constrain pwm to (PWM_MIN, PWM_MAX)
        const float pwm = 0.5f + 0.5f * voltage / m_voltage_max;

        // set duty cycle
        m_FastPWM.write(pwm);

        // update signals
        m_velocity_setpoint = velocity_setpoint;
        m_voltage = voltage;
        m_pwm = pwm;
    }
}

void DCMotor::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}