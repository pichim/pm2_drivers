#include "Servo.h"

const float Servo::MIN_INPUT = 0.02f;
const float Servo::MAX_INPUT = 0.98f;
const uint16_t Servo::DEFAULT_PERIOD_MUS = 20000;

Servo::Servo(PinName Pin) : ServoPin(Pin)
{
    servoEnabled = false;
    Angle = 0;
    Period = DEFAULT_PERIOD_MUS;
}

/**
 * Sets the pwm period.
 * @param _Period period in mus.
 */
void Servo::setPeriod_mus(uint16_t _Period)
{
    Period = _Period;
}

/**
 * Sets the desired angle.
 * @param _Angle  a value between 0...1.
 */
void Servo::setNorlalisedAngle(float _Angle)
{
    if (servoEnabled) {
        if (_Angle < MIN_INPUT) _Angle = MIN_INPUT;
        if (_Angle > MAX_INPUT) _Angle = MAX_INPUT;
        Angle = static_cast<uint16_t>(_Angle * static_cast<float>(Period));
    }
}

void Servo::startPulse()
{
    ServoPin = 1;
    PulseStop.attach(callback(this, &Servo::endPulse), std::chrono::microseconds{static_cast<long int>(Angle)});
}

void Servo::endPulse()
{
    ServoPin = 0;
}

/**
 * Enables the servo with start angle and period.
 * @param _startAngle a value between 0...1.
  */
void Servo::enable(float _startAngle)
{
    servoEnabled = true;
    if (_startAngle < MIN_INPUT) _startAngle = MIN_INPUT;
    if (_startAngle > MAX_INPUT) _startAngle = MAX_INPUT;
    Angle = static_cast<uint16_t>(_startAngle * static_cast<float>(Period));
    Pulse.attach(callback(this, &Servo::startPulse), std::chrono::microseconds{static_cast<long int>(Period)});
}

/**
 * Enables the servo with last set angle and period.
 */
void Servo::enable()
{
    enable(0.0f);
}

/**
 * Disables the servo.
 */
void Servo::disable()
{
    servoEnabled = false;
    Pulse.detach();
}

/**
 * Returns true if Servo is enabled.
 * @return isEnable.
 */
bool Servo::isEnabled()
{
    return servoEnabled;
}