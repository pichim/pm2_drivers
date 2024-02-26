#include "PwmIn.h"

using namespace std::chrono;

PwmIn::PwmIn(PinName pin) : _interuptIn(pin)
{
    _interuptIn.enable_irq();
    _interuptIn.rise(callback(this, &PwmIn::measurePeriodAndResetTimer));
    _interuptIn.fall(callback(this, &PwmIn::measurePulseWidth));
    _timer.start();
}

PwmIn::~PwmIn()
{
}

float PwmIn::getPeriod()
{
    return _period;
}

float PwmIn::getPulsewidth()
{
    return _pulsewidth;
}

float PwmIn::getDutyCycle()
{
    if (_period != 0.0f)
        return _pulsewidth / _period;
    else
        return 0.0f;
}

void PwmIn::measurePeriodAndResetTimer()
{
    const int64_t time_actual_us = duration_cast<microseconds>(_timer.elapsed_time()).count();
    _period = static_cast<float>(time_actual_us - _time_previous_us);
    _time_previous_us = time_actual_us;
}

void PwmIn::measurePulseWidth()
{
    _pulsewidth = static_cast<float>(duration_cast<microseconds>(_timer.elapsed_time()).count() - _time_previous_us);
}
