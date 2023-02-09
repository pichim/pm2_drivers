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
 
#ifndef MBED_PULSE_H
#define MBED_PULSE_H

#include "mbed.h"

/** Pulse Input/Output Class(es)
 */
 
class PulseInOut   {
    public:
        /** Create a PulseInOut object connected to the specified pin
        * @param pin i/o pin to connect to
        */
        PulseInOut(PinName);
        ~PulseInOut();
        /** Set the value of the pin
        * @param val Value to set, 0 for LOW, otherwise HIGH
        */
        void write(int val);
        /** Send a pulse of a given value for a specified time
        * @param val Value to set, 0 for LOW, otherwise HIGH
        * @param time Length of pulse in microseconds
        */
        void write_us(int val, int time);
        /** Return the length of the next HIGH pulse in microsconds
        */
        int read_high_us();
        /** Return the length of the next HIGH pulse in microseconds or -1 if longer than timeout
        * @param timeout Time before pulse reading aborts and returns -1, in microseconds
        */
        int read_high_us(int timeout);
        /** Return the length of the next LOW pulse in microsconds
        */
        int read_low_us();
        /** Return the length of the next LOW pulse in microseconds or -1 if longer than timeout
        * @param timeout Time before pulse reading aborts and returns -1, in microseconds
        */
        int read_low_us(int timeout);
        /** Return the length of the next pulse in microsconds
        */
        int read_us();
        /** Return the length of the next pulse in microseconds or -1 if longer than timeout
        * @param timeout Time before pulse reading aborts and returns -1, in microseconds
        */
        int read_us(int timeout);
    private:
        int startval;
        Timer pulsetime, runtime;
        DigitalInOut io;
};

#endif