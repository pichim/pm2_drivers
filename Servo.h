/* mbed Servo Library without using PWM pins
 * Copyright (c) 2010 Jasper Denkers
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef MBED_SERVO_H
#define MBED_SERVO_H

#include "mbed.h"
#include <cstdint>

class Servo
{

public:

    Servo(PinName Pin);
    void setPeriod_mus(uint16_t _Period);
    void setNorlalisedAngle(float _Angle);
    void enable(float _startAngle);
    void enable();
    void disable();
    bool isEnabled();

private:

    static const float    MIN_INPUT;
    static const float    MAX_INPUT;
    static const uint16_t DEFAULT_PERIOD_MUS;

    void startPulse();
    void endPulse();

    bool servoEnabled;
    uint16_t Angle, Period;
    DigitalOut ServoPin;
    Ticker Pulse;
    Timeout PulseStop;
};

#endif