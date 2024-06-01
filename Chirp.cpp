#include "Chirp.h"

Chirp::Chirp(const float f0, const float f1, const float t1, const float Ts)
{
    init(f0, f1, t1, Ts);
}

// initialize the chirp signal generator
// f0: start frequency in Hz
// f1: end frequency in Hz
// t1: signal length in seconds
// Ts: sampling time in seconds
void Chirp::init(const float f0, const float f1, const float t1, const float Ts)
{
    m_chirp.f0 = f0;
    m_chirp.Ts = Ts;
    m_chirp.N = static_cast<uint32_t>(t1 / Ts);
    m_chirp.beta = powf(f1 / f0, 1.0f / t1);
    m_chirp.k0 = 2.0f * M_PIf / logf(m_chirp.beta);
    m_chirp.k1 = m_chirp.k0 * f0;
    reset();
}

// reset the chirp signal generator fully
void Chirp::reset()
{
    m_chirp.count = 0;
    m_chirp.isFinished = false;
    resetSignals();
}

// update the chirp signal generator
bool Chirp::update()
{
    if (m_chirp.isFinished) {

        return false;

    } else if (m_chirp.count == m_chirp.N) {

        m_chirp.isFinished = true;
        resetSignals();
        return false;

    } else {

        m_chirp.fchirp = m_chirp.f0 * powf(m_chirp.beta, static_cast<float>(m_chirp.count) * m_chirp.Ts);
        m_chirp.sinarg = m_chirp.k0 * m_chirp.fchirp - m_chirp.k1;
        m_chirp.sinarg = fmodf(m_chirp.sinarg, 2.0f * M_PIf);
        m_chirp.exc = sinf(m_chirp.sinarg);
        m_chirp.count++;

        return true;
    }
}

float Chirp::getFreq() const
{
    return m_chirp.fchirp;
}

float Chirp::getSinarg() const
{
    return m_chirp.sinarg;
}

float Chirp::getExc() const
{
    return m_chirp.exc;
}

// reset the chirp signal generator signals
void Chirp::resetSignals()
{
    m_chirp.exc = 0.0f;
    m_chirp.fchirp = 0.0f;
    m_chirp.sinarg = 0.0f;
}