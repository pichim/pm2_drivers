#ifndef AVG_FILTER_H_
#define AVG_FILTER_H_

#include <mbed.h>

/**
 * Average filter class.
 */
class AvgFilter
{
public:
    AvgFilter(){};
    AvgFilter(uint8_t N);
    virtual ~AvgFilter();

    void init(uint8_t N);
    void reset(float val);
    void reset();
    float apply(float inp);
    float getVal() const { return m_val; }

private:
    float m_val;
    uint8_t m_N;
    uint8_t m_idx;
    float *m_ring_buffer;
};

#endif /* AVG_FILTER_H_ */