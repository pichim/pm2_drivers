#ifndef CHIRP_H_
#define CHIRP_H_

#include <math.h>
#include <stdint.h>

#define M_PIf 3.14159265358979323846f /* pi */

class Chirp
{
public:
    Chirp() {};
    Chirp(const float f0, const float f1, const float t1, const float Ts);
    virtual ~Chirp() = default;

    void init(const float f0, const float f1, const float t1, const float Ts);
    void reset();
    bool update();

    float getFreq() const;
    float getSinarg() const;
    float getExc() const;

private:
    struct ChripParams {
        float f0, Ts, beta, k0, k1;
        uint32_t count, N;
        float exc, fchirp, sinarg;
        bool isFinished;
    } m_chirp;

    void resetSignals();
};

#endif /* CHIRP_H_ */
