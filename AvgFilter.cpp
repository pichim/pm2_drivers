#include "AvgFilter.h"

AvgFilter::AvgFilter(uint8_t N)
{
    init(N);
}

AvgFilter::~AvgFilter() {}

void AvgFilter::init(uint8_t N)
{
    m_N = N;
    m_ring_buffer = (float*)malloc(m_N * sizeof(float));
    reset();
}

void AvgFilter::reset(float val)
{
    m_val = val;
    const float scaled_val = val / (float)m_N;
    m_idx = 0;
    for (uint8_t i = 0; i < m_N; i++)
        m_ring_buffer[i] = scaled_val;
}

void AvgFilter::reset()
{
    reset(0.0f);
}

float AvgFilter::apply(float inp)
{
    // remove last value from ringbuffer
    m_val -= m_ring_buffer[m_idx];

    // add new value and update ringbuffer, rinbuffer stores the scaled value
    const float scaled_inp = inp / (float)m_N;
    m_val += scaled_inp;
    m_ring_buffer[m_idx] = scaled_inp;

    // check if we are at the end of the ringbuffer
    m_idx++;
    if (m_idx == m_N)
        m_idx = 0;

    return m_val;
}