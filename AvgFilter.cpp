#include "AvgFilter.h"

AvgFilter::AvgFilter(uint8_t _N)
{
    setup(_N);
}

AvgFilter::~AvgFilter()
{
}

void AvgFilter::setup(uint8_t _N)
{
    m_N = _N;
    m_ring_buffer = (float*)malloc(m_N*sizeof(float));
    reset();
}

void AvgFilter::reset(float _avg)
{
    m_avg = _avg;
    float scaled_inp = m_avg / (float)m_N;
    m_idx = 0;
    for(uint8_t i = 0; i < m_N; i++) {
        m_ring_buffer[i] = scaled_inp;
    }
}

void AvgFilter::reset()
{
    reset(0.0f);
}

float AvgFilter::update(float _inp)
{
    // remove last value from ringbuffer
    m_avg -= m_ring_buffer[m_idx];

    // add new value and update ringbuffer, rinbuffer stores the scaled value
    float scaled_inp = _inp / (float)m_N;
    m_avg += scaled_inp;
    m_ring_buffer[m_idx] = scaled_inp;

    // check if we are at the end of the ringbuffer
    m_idx++;
    if (m_idx == m_N)
    {
        m_idx = 0;
    }

    return m_avg;
}