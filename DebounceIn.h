/*
MIT License

Copyright (c) 2020 Pavel Slama

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef DEBOUNCEIN_H
#define DEBOUNCEIN_H

#include "mbed.h"
#include <chrono>
using namespace std::chrono;

#define DEFAULT_DEBOUNCE 20ms

class DebounceIn {
  public:
    typedef enum {
        None = 0,
        Rise,
        Fall
    } Type;

    DebounceIn(PinName pin, PinMode mode = PullNone);
    void rise(Callback<void()> cb, microseconds timeout = DEFAULT_DEBOUNCE);
    void fall(Callback<void()> cb, microseconds timeout = DEFAULT_DEBOUNCE);
    int32_t read();

    operator int();

  private:
    InterruptIn _interrupt;
    Timeout _timeout[2];
    Callback<void()> _cb[2];
    void intRiseCb();
    void intFallCb();
    void timeoutRiseCb();
    void timeoutFallCb();
    microseconds _timeout_interval[2];
};

#endif
