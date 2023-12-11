#ifndef ULTRASONIC_SENSOR_H_
#define ULTRASONIC_SENSOR_H_

#include "mbed.h"

class UltrasonicSensor
{
public:
    explicit UltrasonicSensor(PinName pin);
    virtual ~UltrasonicSensor();

    // create an operator to access read()
    operator float()
    {
        return read();
    }

    float read();

private:
    DigitalInOut m_digitalInOut;
    InterruptIn m_InteruptIn;
    Timer m_Timer;
    Ticker m_Ticker;
    Timeout m_Timeout;

    float m_gain = 1.7295e-04f;
    float m_offset = 0.02f;
    int m_timeout = 7000;
    int m_pulsetime = 5;
    float m_us_distance_cm = 0.0f;
    bool m_is_new_value = false;

    float getDistance();
    void measure();
    void disableDigitalOutput();
    void startTimer();
    void measureTime();
    void enableDigitalOutput();
    
};
#endif /* ULTRASONIC_SENSOR_H_ */
