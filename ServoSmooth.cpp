#include "ServoSmooth.h"

ServoSmooth::ServoSmooth(PinName pinName) : m_DigitalOut(pinName),
                                            m_Thread(osPriorityHigh),
                                            m_enabled(false),
                                            m_pulse(0.0f)
{
    // set default acceleration profile
    setProfileAcceleration();

    // start thread
    m_Thread.start(callback(this, &ServoSmooth::threadTask));
}

ServoSmooth::~ServoSmooth()
{
    m_Ticker.detach();
    m_Timeout.detach();
    m_Thread.terminate();
}

void ServoSmooth::setProfileAcceleration(float acceleration)
{
    m_Motion.setProfileAcceleration(acceleration);
    m_Motion.setProfileDeceleration(acceleration);
}

void ServoSmooth::setNormalisedPulseWidth(float pulse)
{
    m_pulse = constrainPulse(pulse);
}

void ServoSmooth::enable(float pulse)
{
    m_enabled = true;

    // set pulse width when enabled
    m_pulse = constrainPulse(pulse);
    m_Motion.setPosition(m_pulse);

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &ServoSmooth::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

void ServoSmooth::disable()
{
    m_enabled = false;

    // detach ticker and timeout
    m_Ticker.detach();
    m_Timeout.detach();
}

bool ServoSmooth::isEnabled() const
{
    return m_enabled;
}

void ServoSmooth::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        
        if (isEnabled()) {
            // increment to position
            m_Motion.incrementToPosition(m_pulse, 1.0e-6f * static_cast<float>(PERIOD_MUS));

            // convert to pulse width
            const uint16_t pulse_mus = static_cast<uint16_t>(m_Motion.getPosition() * static_cast<float>(PERIOD_MUS));

            // enable digital output and attach disableDigitalOutput() to timeout for soft PWM
            enableDigitalOutput();
            m_Timeout.attach(callback(this, &ServoSmooth::disableDigitalOutput), std::chrono::microseconds{pulse_mus});
        }
    }
}

void ServoSmooth::enableDigitalOutput()
{
    // set the digital output to high
    m_DigitalOut = 1;
}

void ServoSmooth::disableDigitalOutput()
{
    // set the digital output to low
    m_DigitalOut = 0;
}

void ServoSmooth::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}

float ServoSmooth::constrainPulse(float pulse) const
{
    // constrain pulse to range (INPUT_MIN, INPUT_MAX)
    if (pulse < INPUT_MIN) {
        pulse = INPUT_MIN;
    } else if (pulse > INPUT_MAX) {
        pulse = INPUT_MAX;
}
