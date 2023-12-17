#include <math.h>
#define M_PI 3.14159265358979323846264338327950288

#include "DCMotor.h"

DCMotor::DCMotor(PinName pin_pwm,
                 PinName pin_enc_a,
                 PinName pin_enc_b,
                 float gear_ratio,
                 float kn,
                 float voltage_max,
                 float counts_per_turn) : m_FastPWM(pin_pwm),
                                          m_EncoderCounter(pin_enc_a, pin_enc_b),
                                          m_Thread(osPriorityHigh, 4096)
{
    // motor parameters
    m_counts_per_turn = gear_ratio * counts_per_turn;
    m_voltage_max = voltage_max;
    m_velocity_max = kn / 60.0f * voltage_max;

    // default controller parameters
    const float k_gear = gear_ratio / 78.125f;
    setVelocityCntrl(DCMotor::KP * k_gear, DCMotor::KI * k_gear, DCMotor::KD * k_gear);
    if (kn != 0.0f)
        m_PID_Cntrl_velocity.setCoeff_F(60.0f / kn);
    setRotationCntrlGain();

    // iir filter
    m_IIR_Filter_velocity.setup(2.0f * M_PI * 15.0f,
                                1.0f,
                                TS,
                                1.0f);

    // initialise control signals
    m_count = m_count_previous = m_EncoderCounter.read();
    m_rotation_initial = static_cast<float>(m_count) / m_counts_per_turn;
    m_rotation_target = m_rotation_initial;
    m_rotation_setpoint = m_rotation_initial;
    m_rotation = m_rotation_initial;
    m_velocity_target = 0.0f;
    m_velocity_setpoint = 0.0f;
    m_velocity = 0.0f;
    m_voltage = 0.0f;
    m_pwm = 0.0f;

    m_Motion.setPosition(0.0f);
    m_Motion.setProfileVelocity(m_velocity_max);
    setMaxAcceleration(390.625f / gear_ratio);

    const float fMin = 1.0f;
    const float fMax = 0.99f/2.0f/TS;
    const uint16_t NfexcDes = 80;
    const float Aexc0 =       0.8f * 2.0f * 1.0f;
    const float Aexc1 = 0.5 * 0.8f * 2.0f * 1.0f; // Aexc0/fMax;
    const int   NperMin = 3;
    const float TmeasMin = 0.5f;
    const int   NmeasMin = (int)ceilf(TmeasMin/TS);
    const float Tstart = 1.0f;
    const int   Nstart = (int)ceilf(Tstart/TS);
    const float Tsweep = 0.3f;
    const int   Nsweep = (int)ceilf(Tsweep/TS);
    m_GPA.setParameters(fMin, fMax, NfexcDes, NperMin, NmeasMin, TS, Aexc0, Aexc1, Nstart, Nsweep, true, false);

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

void DCMotor::setVelocityCntrl(float kp, float ki, float kd)
{
    const float tau_f = 1.0f / (2.0f * M_PI * 30.0f);
    const float tau_ro = 1.0f / (2.0f * M_PI * 0.5f / (2.0f * TS));
    m_PID_Cntrl_velocity.setup(kp,
                               ki,
                               kd,
                               tau_f,
                               tau_ro,
                               TS,
                               m_voltage_max * (2.0f * PWM_MIN - 1.0f),
                               m_voltage_max * (2.0f * PWM_MAX - 1.0f));
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
    // print some gpa info
    // m_GPA.printGPAmeasPara();

    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        static float exc = 0.0f;

        // update counts (avoid overflow)
        const short count_actual = m_EncoderCounter.read();
        const short count_delta = count_actual - m_count_previous; // avoid overflow
        m_count_previous = count_actual;

        // update rotation
        m_count += count_delta;
        m_rotation = static_cast<float>(m_count) / m_counts_per_turn;

        // update velocity
        const float rotation_increment = static_cast<float>(count_delta) / m_counts_per_turn;
        m_velocity = m_IIR_Filter_velocity.filter(rotation_increment / TS);

        float velocity_setpoint = 0.0f;
        switch (m_cntrlMode) {
            case CntrlMode::Rotation:
                // increment motion planner to position
                m_Motion.incrementToPosition(m_rotation_target, TS);
                m_rotation_setpoint = m_Motion.getPosition();
                // only set velocity_setpoint if abs of the rotation error is greater than ROTATION_ERROR_MAX
                if (fabs(m_rotation_setpoint - m_rotation) > ROTATION_ERROR_MAX)
                    velocity_setpoint = m_p * (m_rotation_setpoint - m_rotation);
                break;

            case CntrlMode::Velocity:
                // increment motion planner to velocity
                m_Motion.incrementToVelocity(m_velocity_target, TS);
                velocity_setpoint = m_Motion.getVelocity();
                // velocity_setpoint = m_velocity_target;
                break;

            default:
                break; // should not happen
        }

        // constrain velocity to (-m_velocity_max, m_velocity_max)
        velocity_setpoint = (velocity_setpoint >  m_velocity_max) ?  m_velocity_max :
                            (velocity_setpoint < -m_velocity_max) ? -m_velocity_max :
                             velocity_setpoint;

        const float voltage = m_PID_Cntrl_velocity.update(velocity_setpoint,       // w
                                                          m_velocity,              // y_p
                                                          rotation_increment / TS, // y_i
                                                          m_velocity);             // y_d
        // const float voltage = m_PID_Cntrl_velocity.update(velocity_setpoint - m_velocity + exc);

        // calculate pwm and write output
        const float pwm = 0.5f + 0.5f * voltage / m_voltage_max;
        m_FastPWM.write(pwm);

        // update signals
        m_velocity_setpoint = velocity_setpoint;
        m_voltage = voltage;
        m_pwm = pwm;

        // gpa
        // exc = m_GPA(voltage, m_velocity);
    }
}

void DCMotor::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}