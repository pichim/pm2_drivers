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
 
 
 #include "RangeFinder.h"
 
 
using namespace std;

RangeFinder::RangeFinder(PinName pin, int pulsetime, float scale, float offset, int time):
    pio(pin), scale(scale), offset(offset), pulsetime(pulsetime), timeout(time)  {
}

RangeFinder::RangeFinder(PinName pin, float scale, float offset, int time):
    pio(pin), scale(scale), offset(offset), pulsetime(10), timeout(time)  {
}

RangeFinder::~RangeFinder() {
}

float RangeFinder::read_cm()  {
    pio.write_us(1, pulsetime);
    float t = (float) pio.read_high_us(timeout);
    if (t == -1.0)   {
        return -1.0;
    }
    return 100.0f*( t / scale + offset );
}
