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
  
#ifndef MBED_RANGEFINDER_H
#define MBED_RANGEFINDER_H

#include "Pulse.h"

/** Range Finder class
*/

class RangeFinder
{
    public:
        /** Create a RangeFinder object
        * @param pin Digital I/O pin the range finder is connected to.
        * @param pulsetime Time of pulse to send to the rangefinder to trigger a measurement, in microseconds.
        * @param scale Scaling of the range finder's output pulse from microseconds to meters.
        * @param offset Offset of the range finder's output pulse to adjust absolut reference.
        * @param timeout Time to wait for a pulse from the range finder before giving up.
        *        y = x/scale + offset
        */
        RangeFinder(PinName pin, int pulsetime, float scale, float offset, int time);
        RangeFinder(PinName pin, float scale, float offset, int time);
        /** Return the distance to the nearest object, or -1.0 if reading the pulse timed out.
        */
        float operator()() {
            return read_cm();
        }
        
        virtual ~RangeFinder();
        
        float read_cm();
        
    private:
    
        PulseInOut pio;
        
        float scale, offset;
        int pulsetime, timeout;
        
};

#endif