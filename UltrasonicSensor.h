#ifndef ULTRASONIC_SENSOR_H_
#define ULTRASONIC_SENSOR_H_

#include "mbed.h"

#include "ThreadFlag.h"

// Time (mus), Distance (cm)
//     10000 ,        164
//     14000 ,        232.5
// ->  12105 ,        200
//     12000 ,        198.1  (measured)


class UltrasonicSensor
{
public:
    explicit UltrasonicSensor(PinName pin);
    virtual ~UltrasonicSensor();

    // create an operator to access read()
    operator float() {
        return read_cm();
    }

    float read_cm();

private:
    static constexpr int64_t PERIOD_MUS = 12000;

    DigitalInOut m_DigitalInOut;
    InterruptIn m_InteruptIn;
    Timer m_Timer;
    Timeout m_Timeout;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    float m_gain = 0.0170971;
    float m_offset = 1.7451288f;

    float m_us_distance_cm = 0.0f;
    bool m_is_new_value = false;

    void stopPulseAndWaitForRisingEdge();
    void startTimerAndWaitForFallingEdge();
    void measureTimeAndUpdateDistance();

    void threadTask();
    void sendThreadFlag(); 
};
#endif /* ULTRASONIC_SENSOR_H_ */
