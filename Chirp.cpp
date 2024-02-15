#include <math.h>
#define M_PI 3.14159265358979323846264338327950288

#include "Chirp.h"

Chirp::Chirp(float f0, float f1, float t1, float Ts)
{
    init(f0, f1, t1, Ts);
}

// initialize the chirp signal generator
// f0: start frequency in Hz
// f1: end frequency in Hz
// t1: signal length in seconds
// Ts: sampling time in seconds
void Chirp::init(float f0, float f1, float t1, float Ts)
{
    m_chirp.f0 = f0;
    m_chirp.Ts = Ts;
    m_chirp.N = static_cast<uint32_t>(t1 / Ts);
    m_chirp.beta = powf(f1 / f0, 1.0f / t1);
    m_chirp.k0 = 2.0f * M_PI / logf(m_chirp.beta);
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

        // wrap sinarg to 0...2*pi
        m_chirp.sinarg = fmodf(m_chirp.sinarg, 2.0f * M_PI);

        // use cosine so that the angle will oscillate around 0 (integral of gyro)
        m_chirp.exc = sinf(m_chirp.sinarg);
        
        // // frequencies below 1 Hz will lead to the same angle magnitude as at 1 Hz (integral of gyro)
        // if (m_chirp.fchirp < 1.0f) {
        //     m_chirp.exc = m_chirp.fchirp * m_chirp.exc;
        // }
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