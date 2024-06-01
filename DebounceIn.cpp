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

#include "DebounceIn.h"

DebounceIn::DebounceIn(PinName pin, PinMode mode) :
    _interrupt(pin, mode) {
    _cb[0] = NULL;
    _cb[1] = NULL;
}


void DebounceIn::rise(Callback<void()> cb, microseconds timeout) {
    if (cb) {
        _cb[0] = cb;
        _timeout_interval[0] = timeout;
        _interrupt.rise(callback(this, &DebounceIn::intRiseCb));

    } else {
        _cb[0] = NULL;
        _interrupt.rise(NULL);
        _timeout[0].detach();
    }
}

void DebounceIn::intRiseCb() {
    _timeout[0].attach(callback(this, &DebounceIn::timeoutRiseCb), _timeout_interval[0]);
    _interrupt.rise(NULL);  // detach interrupt
}

void DebounceIn::timeoutRiseCb() {
    if (read()) {
        if (_cb[0]) {
            _cb[0].call();
        }
    }

    _interrupt.rise(callback(this, &DebounceIn::intRiseCb));
}


void DebounceIn::fall(Callback<void()> cb, microseconds timeout) {
    if (cb) {
        _cb[1] = cb;
        _timeout_interval[1] = timeout;
        _interrupt.fall(callback(this, &DebounceIn::intFallCb));

    } else {
        _cb[1] = NULL;
        _interrupt.fall(NULL);
        _timeout[1].detach();
    }
}

void DebounceIn::timeoutFallCb() {
    if (!read()) {
        if (_cb[1]) {
            _cb[1].call();
        }
    }

    _interrupt.fall(callback(this, &DebounceIn::intFallCb));
}

void DebounceIn::intFallCb() {
    _timeout[1].attach(callback(this, &DebounceIn::timeoutFallCb), _timeout_interval[1]);
    _interrupt.fall(NULL);  // detach interrupt
}


int32_t DebounceIn::read() {
    return _interrupt.read();
}

DebounceIn::operator int() {
    return read();
}
