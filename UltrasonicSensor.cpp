#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(PinName pin) : m_digitalInOut(pin), m_InteruptIn(pin)
{
    m_Timer.start();
    measure();
}

UltrasonicSensor::~UltrasonicSensor()
{
    m_Timeout.detach();
}

float UltrasonicSensor::read()
{
    measure();
    return getDistance();
}

float UltrasonicSensor::getDistance()
{
    if (m_is_new_value) {
        m_is_new_value = false;
        if (m_us_distance_cm < 400.0f)
            return m_us_distance_cm;
        else
            return -1.0f;
    } else {
        return -1.0f;
    }
}

void UltrasonicSensor::measure()
{
    // detach interrupt
    m_InteruptIn.fall(NULL);
    m_InteruptIn.rise(NULL);
    m_InteruptIn.disable_irq();

    // enable digital output and attach disableDigitalOutput() to timeout for soft PWM
    m_digitalInOut.output();
    enableDigitalOutput();
    m_Timeout.attach(callback(this, &UltrasonicSensor::disableDigitalOutput), std::chrono::microseconds{m_pulsetime});
}

void UltrasonicSensor::disableDigitalOutput()
{
    // set the digital output to low
    m_digitalInOut = 0;

    m_digitalInOut.input();
    m_InteruptIn.enable_irq();
    m_InteruptIn.rise(callback(this, &UltrasonicSensor::startTimer));
}

void UltrasonicSensor::startTimer()
{
    m_Timer.reset();
    m_InteruptIn.fall(callback(this, &UltrasonicSensor::measureTime));
}

void UltrasonicSensor::measureTime()
{
    const int pulse_time = std::chrono::duration_cast<std::chrono::microseconds>(m_Timer.elapsed_time()).count();
    m_us_distance_cm = 100.0f * (m_gain * static_cast<float>(pulse_time) + m_offset);
    m_is_new_value = true;
}

void UltrasonicSensor::enableDigitalOutput()
{
    // set the digital output to high
    m_digitalInOut = 1;
}

