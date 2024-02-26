#ifndef PWM_IN_H_
#define PWM_IN_H_

#include "mbed.h"

class PwmIn
{
public:
    explicit PwmIn(PinName pin);
    virtual ~PwmIn();

    float getPeriod();
    float getPulsewidth();
    float getDutyCycle();

protected:
    InterruptIn _interuptIn;
    Timer _timer;

    float _pulsewidth{0.0f};
    float _period{0.0f};
    int64_t _time_previous_us{0};

    void measurePeriodAndResetTimer();
    void measurePulseWidth();
};

#endif /* Serial_Stream_H_ */