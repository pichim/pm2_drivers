/* Copyright (c) 2012 Nick Ryder, University of Oxford
 * nick.ryder@physics.ox.ac.uk
 *
 *  MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 * and associated documentation files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
 
 
#include "Pulse.h"

PulseInOut::PulseInOut(PinName pin):
        startval(0), pulsetime(), runtime(), io(pin)    {
}


PulseInOut::~PulseInOut() {
}

void PulseInOut::write(int val) {
    io.output();
    io = val;
}

void PulseInOut::write_us(int val, int time)   {
    io.output();
    io = val;
    wait_us(time);
    io = !val;
}

int PulseInOut::read_high_us()  {
    pulsetime.reset();
    io.input();
    while (io == 1) {
    }
    while (io == 0) {
    }
    pulsetime.start();
    while (io == 1) {
    }
    pulsetime.stop();
    return std::chrono::duration_cast<std::chrono::microseconds>(pulsetime.elapsed_time()).count();
}

int PulseInOut::read_high_us(int timeout)  {
    runtime.reset();
    runtime.start();
    pulsetime.reset();
    io.input();
    while (io == 1) {
        if (std::chrono::duration_cast<std::chrono::microseconds>(runtime.elapsed_time()).count() > timeout)   return -1;
    }
    while (io == 0) {
        if (std::chrono::duration_cast<std::chrono::microseconds>(runtime.elapsed_time()).count() > timeout)   return -1;
    }
    pulsetime.start();
    while (io == 1) {
        if (std::chrono::duration_cast<std::chrono::microseconds>(runtime.elapsed_time()).count() > timeout)   return -1;
    }
    pulsetime.stop();
    return std::chrono::duration_cast<std::chrono::microseconds>(pulsetime.elapsed_time()).count();
}

int PulseInOut::read_low_us()   {
    pulsetime.reset();
    io.input();
    while (io == 0) {
    }
    while (io == 1) {
    }
    pulsetime.start();
    while (io == 0) {
    }
    pulsetime.stop();
    return std::chrono::duration_cast<std::chrono::microseconds>(pulsetime.elapsed_time()).count();
}

int PulseInOut::read_low_us(int timeout)   {
    runtime.reset();
    runtime.start();
    pulsetime.reset();
    io.input();
    while (io == 0) {
        if (std::chrono::duration_cast<std::chrono::microseconds>(runtime.elapsed_time()).count() > timeout)   return -1;
    }
    while (io == 1) {
        if (std::chrono::duration_cast<std::chrono::microseconds>(runtime.elapsed_time()).count() > timeout)   return -1;
    }
    pulsetime.start();
    while (io == 0) {
        if (std::chrono::duration_cast<std::chrono::microseconds>(runtime.elapsed_time()).count() > timeout)   return -1;
    }
    pulsetime.stop();
    return std::chrono::duration_cast<std::chrono::microseconds>(pulsetime.elapsed_time()).count();
}

int PulseInOut::read_us()  {
    pulsetime.reset();
    io.input();
    startval = io;
    while (io == startval)   {
    }
    pulsetime.start();
    while (io != startval)  {
    }
    pulsetime.stop();
    return std::chrono::duration_cast<std::chrono::microseconds>(pulsetime.elapsed_time()).count();
}

int PulseInOut::read_us(int timeout)   {
    runtime.reset();
    runtime.start();
    pulsetime.reset();
    io.input();
    startval = io;
    while (io == startval)  {
        if (std::chrono::duration_cast<std::chrono::microseconds>(runtime.elapsed_time()).count() > timeout)   return -1;
    }
    pulsetime.start();
    while (io != startval)   {
        if (std::chrono::duration_cast<std::chrono::microseconds>(runtime.elapsed_time()).count() > timeout)   return -1;
    }
    pulsetime.stop();
    return std::chrono::duration_cast<std::chrono::microseconds>(pulsetime.elapsed_time()).count();
}