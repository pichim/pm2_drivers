#include "Servo.h"

const float Servo::MIN_INPUT = 0.01f;
const float Servo::MAX_INPUT = 0.99f;

Servo::Servo(PinName Pin) : ServoPin(Pin)
{
    servoEnabled = false;
    Position = 0;
    Period = 0;
}

/**
 * Sets the pwm period.
 * @_Period period in mus.
 */
void Servo::SetPeriod(float _Period)
{
    Period = _Period;
}

/**
 * Sets the desired angle.
 * @_Input a value between 0...1.
 */
void Servo::SetPosition(float _Input)
{
    if (servoEnabled) {
        if (_Input < MIN_INPUT) _Input = MIN_INPUT;
        if (_Input > MAX_INPUT) _Input = MAX_INPUT;
        Position = static_cast<int>(_Input * static_cast<float>(Period));
    }
}

void Servo::StartPulse()
{
    ServoPin = 1;
    PulseStop.attach(callback(this, &Servo::EndPulse), std::chrono::microseconds{static_cast<long int>(Position)});
}

void Servo::EndPulse()
{
    ServoPin = 0;
}

/**
 * Enables the servo with start angle and period.
 * @_StartInput a value between 0...1.
 * @_Period period in mus.
 */
void Servo::Enable(float _StartInput, int _Period)
{
    servoEnabled = true;
    if (_StartInput < MIN_INPUT) _StartInput = MIN_INPUT;
    if (_StartInput > MAX_INPUT) _StartInput = MAX_INPUT;
    Period = _Period;
    Position = static_cast<int>(_StartInput * static_cast<float>(Period));
    Pulse.attach(callback(this, &Servo::StartPulse), std::chrono::microseconds{static_cast<long int>(Period)});
}

/**
 * Enables the servo with last set angle and period.
 */
void Servo::Enable()
{
    servoEnabled = true;
    Pulse.attach(callback(this, &Servo::StartPulse), std::chrono::microseconds{static_cast<long int>(Period)});
}

/**
 * Disables the servo.
 */
void Servo::Disable()
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