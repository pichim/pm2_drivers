#ifndef CHIRP_H_
#define CHIRP_H_

#include <stdint.h>

class Chirp
{
public:
    Chirp(){};
    Chirp(float f0, float f1, float t1, float Ts);

    void init(float f0, float f1, float t1, float Ts);
    void reset();
    bool update();

    float getFreq() const;
    float getSinarg() const;
    float getExc() const;

private:
    struct chrip {
        float f0, Ts, beta, k0, k1;
        uint32_t count, N;
        float exc, fchirp, sinarg;
        bool isFinished;
    } m_chirp;

    void resetSignals();
};

#endif /* CHIRP_H_ */
