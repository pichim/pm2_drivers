#ifndef PWM_IN_H_
#define PWM_IN_H_

#include "mbed.h"

class PwmIn
{
public:
    explicit PwmIn(PinName pin);
    virtual ~PwmIn();

    uint32_t getPeriod();
    uint32_t getPulsewidth();
    float getDutyCycle();
    void invertPolarity();

protected:
    InterruptIn _interuptIn;
    Timer _timer;

    uint32_t _pulsewidth{0};
    uint32_t _period{0};
    uint32_t _time_previous_us{0};

    void measurePeriodAndResetTimer();
    void measurePulseWidth();
};

#endif /* Serial_Stream_H_ */