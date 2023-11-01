#ifndef AVGFILTER_H_
#define AVGFILTER_H_

#include <mbed.h>

class AvgFilter
{
public:
    AvgFilter(){};
    AvgFilter(uint8_t N);
    virtual ~AvgFilter();

    void Init(uint8_t N);
    void Reset(float val);
    void Reset();
    float Apply(float inp);

private:
    float m_val;
    uint8_t m_N;
    uint8_t m_idx;
    float *m_ring_buffer;
};

#endif /* AVGFILTER_H_ */