#ifndef AVGFILTER_H_
#define AVGFILTER_H_

#include <mbed.h>

class AvgFilter
{
public:
    AvgFilter(){};
    AvgFilter(uint8_t _N);
    virtual ~AvgFilter();

    void setup(uint8_t _N);
    void reset(float _avg);
    void reset();
    float update(float _inp);

private:
    float m_avg;
    uint8_t m_N;
    uint8_t m_idx;
    float *m_ring_buffer;
};

#endif /* AVGFILTER_H_ */