/*
 * LowpassFilter.h
 * Copyright (c) 2018, ZHAW
 * All rights reserved.
 */

#ifndef LOWPASS_FILTER_H_
#define LOWPASS_FILTER_H_

#include <cstdlib>

/**
 * This class implements a time-discrete 2nd order lowpass filter for a series of data values.
 * This filter can typically be used within a periodic task that takes measurements that need
 * to be filtered, like speed or position values.
 */
class LowpassFilter
{

public:

    LowpassFilter();
    virtual ~LowpassFilter();
    void    reset();
    void    reset(float value);
    void    setPeriod(float period);
    void    setFrequency(float frequency);
    float   getFrequency();
    float   filter(float value);

private:

    float   period;
    float   frequency;
    float   a11, a12, a21, a22, b1, b2;
    float   x1, x2;
};

#endif /* LOWPASS_FILTER_H_ */


