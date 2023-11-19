#include "Servo.h"

Servo::Servo(PinName pinName) : m_DigitalOut(pinName),
                                m_Ticker(),
                                m_Timeout(),
                                m_servo_enabled(false),
                                m_pulse_mus(0)
{
}

Servo::~Servo()
{
    m_Ticker.detach();
}

void Servo::setNormalisedPulseWidth(float pulse)
{
    if (isEnabled()) {
        m_pulse_mus = static_cast<uint16_t>(constrainPulse(pulse) * static_cast<float>(PERIOD_MUS));
    }
}

void Servo::enable(float pulse)
{
    m_servo_enabled = true;
    setNormalisedPulseWidth(pulse);
    m_Ticker.attach(callback(this, &Servo::startPulse), std::chrono::microseconds{PERIOD_MUS});
}

void Servo::disable()
{
    m_servo_enabled = false;
    m_Ticker.detach();
}

bool Servo::isEnabled() const
{
    return m_servo_enabled;
}

float Servo::constrainPulse(float pulse) const
{
    // check if pulse is within range
    if (pulse < INPUT_MIN) 
        pulse = INPUT_MIN;
    else if (pulse > INPUT_MAX)
        pulse = INPUT_MAX;
    return pulse;
}

void Servo::startPulse()
{
    // enable digital output and attach endPulse() to timeout
    m_DigitalOut = 1;
    m_Timeout.attach(callback(this, &Servo::endPulse), std::chrono::microseconds{m_pulse_mus});
}

void Servo::endPulse()
{
    // diable digital output
    m_DigitalOut = 0;
}