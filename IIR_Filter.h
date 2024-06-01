#ifndef IIR_Filter_H_
#define IIR_Filter_H_

#include <math.h>

class IIR_Filter
{
public:
    IIR_Filter(float T, float Ts);                                          // G(s) = s/(T*s + 1)
    IIR_Filter(float T, float Ts, float K);                                 // G(s) = K/(T*s + 1)
    IIR_Filter(float w0, float D, float Ts, float K);                       // G(s) = K*w0^2/(s^2 + 2*D*w0*s + w0^2)
    IIR_Filter(float wz, float Dz, float wp, float Dp, float Ts, float K);  // G(s) = K * wp^2/wz^2 * (s^2 + 2*Dz*wz*s + wz^2)/(s^2 + 2*Dp*wp*s + wp^2)
    IIR_Filter(float k, float Tz, float Tp, float Ts, int typ);             // G(s) = K * (Tz*s+1)/(Tp*s+1) Lead/Lag in bilinear trafo
    IIR_Filter(float *b, float *a, int nb, int na);                         // G(z) = B(z) / A(z) = (b_0 + b_1*z^-1 + ... + b_nb*z^-nb) / (1 + a_1*z^-1 + ... + a_na*z^-na)
    IIR_Filter(float *b, float *a, int nb, int na, float yMin, float yMax); // G(z) = B(z) / A(z) = (b_0 + b_1*z^-1 + ... + b_nb*z^-nb) / (1 + a_1*z^-1 + ... + a_na*z^-na)

    IIR_Filter(){};

    virtual ~IIR_Filter();

    void setup(float T, float Ts);                                          // G(s) = s/(T*s + 1)
    void setup(float T, float Ts, float K);                                 // G(s) = K/(T*s + 1)
    void setup(float w0, float D, float Ts, float K);                       // G(s) = K*w0^2/(s^2 + 2*D*w0*s + w0^2)
    void setup(float wz, float Dz, float wp, float Dp, float Ts, float K);  // G(s) = K * wp^2/wz^2 * (s^2 + 2*Dz*wz*s + wz^2)/(s^2 + 2*Dp*wp*s + wp^2)
    void setup(float k, float Tz, float Tp, float Ts, int typ);             // G(s) = K * (Tz*s+1)/(Tp*s+1) Lead/Lag in bilinear trafo
    void setup(float *b, float *a, int nb, int na);                         // G(z) = B(z) / A(z) = (b_0 + b_1*z^-1 + ... + b_nb*z^-nb) / (1 + a_1*z^-1 + ... + a_na*z^-na)
    void setup(float *b, float *a, int nb, int na, float yMin, float yMax); // G(z) = B(z) / A(z) = (b_0 + b_1*z^-1 + ... + b_nb*z^-nb) / (1 + a_1*z^-1 + ... + a_na*z^-na)

    void reset();
    void reset(float u);
    void reset(float u, float y);

    void setLimits(float yMax = 1.0e6f);
    void setLimits(float yMin, float yMax);

    float filter(float u);

    float get_output();

    float prewarp(float T, float Ts);

    // void print_filter_coeff();
    float *B;
    float *A;
    unsigned nb, na;

private:
    float *uk;
    float *yk;
    float K;
    float yMin, yMax;

    float saturate(float y, float yMin, float yMax);
};
#endif