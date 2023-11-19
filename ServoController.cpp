#include "ServoController.h"

ServoController::ServoController(PinName pinName) : m_Servo(pinName),
                                                    m_Thread(osPriorityHigh, 4096),
                                                    m_pulse(0.0f)
{
    // set up motion
    m_Motion.setProfileVelocity(PROFILE_VELOCITY);
    m_Motion.setProfileAcceleration(PROFILE_ACCELERATION);
    m_Motion.setProfileDeceleration(PROFILE_ACCELERATION);

    // set up thread
    m_Thread.start(callback(this, &ServoController::run));
    m_Ticker.attach(callback(this, &ServoController::sendThreadFlag), std::chrono::microseconds{Servo::PERIOD_MUS});
}

ServoController::~ServoController()
{
    m_Servo.disable();
    m_Ticker.detach();
    m_Thread.terminate();
}

void ServoController::setNormalisedPulseWidth(float pulse)
{
    if (isEnabled()) {
        m_pulse = m_Servo.constrainPulse(pulse);
    }
}

void ServoController::enable(float pulse)
{
    m_pulse = m_Servo.constrainPulse(pulse);
    m_Motion.setPosition(m_pulse);
    m_Servo.enable(m_pulse);
}

void ServoController::disable()
{
    m_Servo.disable();
}

bool ServoController::isEnabled() const
{
    return m_Servo.isEnabled();
}

void ServoController::run()
{
    Timer timer;
    timer.start();
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        if (isEnabled()) {
            m_Motion.incrementToPosition(m_pulse, TS);
            const float pulse_smooth = m_Motion.getPosition();
            const float pulse_smooth_velocity = m_Motion.getVelocity();
            const int time_mus = std::chrono::duration_cast<std::chrono::microseconds>(timer.elapsed_time()).count();
            printf("%3d, %3.6f, %3.6f, %3.6f\n",
                    time_mus,
                    m_pulse,
                    pulse_smooth,
                    pulse_smooth_velocity);
            m_Servo.setNormalisedPulseWidth(pulse_smooth);
        }
    }
}

void ServoController::sendThreadFlag()
{
    m_Thread.flags_set(m_ThreadFlag);
}