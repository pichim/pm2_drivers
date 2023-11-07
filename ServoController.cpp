#include "ServoController.h"

const float ServoController::TS = 0.02f;

ServoController::ServoController(PinName pinName) : m_Servo(pinName),
                                                    m_Thread(osPriorityHigh, 4096)
{
    // set up motion
    m_Motion.setProfileVelocity(1.0e6f); // 1.0e6f instead of inf
    float max_acceleration = 0.35f;
    m_Motion.setProfileAcceleration(max_acceleration);
    m_Motion.setProfileDeceleration(max_acceleration);
    
    // set up thread
    m_Thread.start(callback(this, &ServoController::run));
    m_Ticker.attach(callback(this, &ServoController::sendThreadFlag), std::chrono::microseconds{(int64_t)(1.0e6f * TS)});
}

ServoController::~ServoController()
{
    m_Servo.disable();
    m_Ticker.detach();
}

void ServoController::setNormalisedPulseWidth(float pulse)
{
    if (m_Servo.isEnabled()) {
        m_pulse = m_Servo.constrainPulse(pulse);
    }
}

void ServoController::enable(float pulse)
{
    m_pulse = m_Servo.constrainPulse(pulse);
    m_Motion.setPosition(m_pulse);
    m_Servo.enable(m_pulse);
}

void ServoController::enable()
{
    enable(0.0f);
}

void ServoController::disable()
{
    m_Servo.disable();
}

bool ServoController::isEnabled()
{
    return m_Servo.isEnabled();
}

void ServoController::run()
{
    Timer timer;
    timer.start();
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        if (m_Servo.isEnabled()) {
            // only update motion if servo is enabled
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