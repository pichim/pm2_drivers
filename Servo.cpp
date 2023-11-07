#include "Servo.h"

const float Servo::INPUT_MIN = 0.01f;
const float Servo::INPUT_MAX = 0.99f;
const uint16_t Servo::PERIOD_MUS = 20000;

Servo::Servo(PinName pinName) : m_DigitalOut(pinName)
{
    m_servo_enabled = false;

    // set default pulse width and period
    m_pulse_mus = 0;
    m_period_mus = PERIOD_MUS;
}

Servo::~Servo()
{
    m_Ticker.detach();
}

void Servo::setNormalisedPulseWidth(float pulse)
{
    if (m_servo_enabled) {
        // check if pulse is within range
        pulse = constrainPulse(pulse);
        
        // update pulse width as fraction of period in mus
        m_pulse_mus = (uint16_t)(pulse * (float)(m_period_mus));
    }
}

void Servo::enable(float pulse)
{
    m_servo_enabled = true;
    setNormalisedPulseWidth(pulse);
    // attach startPulse() to ticker
    m_Ticker.attach(callback(this, &Servo::startPulse), std::chrono::microseconds{m_period_mus});
}

void Servo::enable()
{
    enable(0.0f);
}

void Servo::disable()
{
    m_servo_enabled = false;
    m_Ticker.detach();
}

bool Servo::isEnabled()
{
    return m_servo_enabled;
}

float Servo::constrainPulse(float pulse)
{
    // check if pulse is within range
    pulse = (pulse < INPUT_MIN) ? INPUT_MIN : pulse;
    pulse = (pulse > INPUT_MAX) ? INPUT_MAX : pulse;
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