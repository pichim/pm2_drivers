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

uint32_t PwmIn::getPeriod()
{
    return _period;
}

uint32_t PwmIn::getPulsewidth()
{
    return _pulsewidth;
}

float PwmIn::getDutyCycle()
{
    const float period = static_cast<float>(_period);
    if (period != 0.0f)
        return static_cast<float>(_pulsewidth) / period;
    else
        return 0.0f;
}

void PwmIn::measurePeriodAndResetTimer()
{
    const uint32_t time_actual_us = duration_cast<microseconds>(_timer.elapsed_time()).count();
    _period = time_actual_us - _time_previous_us;
    _time_previous_us = time_actual_us;
}

void PwmIn::measurePulseWidth()
{
    _pulsewidth = duration_cast<microseconds>(_timer.elapsed_time()).count() - _time_previous_us;
}

void PwmIn::invertPolarity()
{
    _interuptIn.disable_irq();
    _pulsewidth = 0;
    _period = 0;
    _interuptIn.rise(callback(this, &PwmIn::measurePulseWidth));
    _interuptIn.fall(callback(this, &PwmIn::measurePeriodAndResetTimer));
    _interuptIn.enable_irq();
}
