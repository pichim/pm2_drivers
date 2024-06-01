#include "IIR_Filter.h"

/*
    IIR filter implemention for the following filter types:
    init for: - first order differentiatior:     G(s) = s/(T*s + 1)
              - first order lowpass with gain:   G(s) = K/(T*s + 1)
              - second order lowpass with gain:  G(s) = K*w0^2/(s^2 + 2*D*w0*s + w0^2)
              - compl. conj. lead lag with gain: G(s) = K*w0p^2/w0z^2 * (s^2 + 2*Dz*w0z*s + w0z^2)/(s^2 + 2*Dp*w0p*s + w0p^2)
              - nth order, with arbitrary values
    - billinear transformation is used for s -> z
    - reseting the filter only makes sense for static signals, whatch out if you're using the differentiator, static corresponds to output zero
    - you need to be carefull with with small sampling times, since the filter is running in float prescision
*/

// G(s) = s/(T*s + 1)
IIR_Filter::IIR_Filter(float T, float Ts)
{
    setup(T, Ts);
}

// G(s) = K/(T*s + 1)
IIR_Filter::IIR_Filter(float T, float Ts, float K)
{
    setup(T, Ts, K);
}

// G(s) = K*w0^2/(s^2 + 2*D*w0*s + w0^2)
IIR_Filter::IIR_Filter(float w0, float D, float Ts, float K)
{
    setup(w0, D, Ts, K);
}

// G(s) = K * wp^2/wz^2 * (s^2 + 2*Dz*wz*s + wz^2)/(s^2 + 2*Dp*wp*s + wp^2)
IIR_Filter::IIR_Filter(float wz, float Dz, float wp, float Dp, float Ts, float K)
{
    setup(wz, Dz, wp, Dp, Ts, K);
}
// G(s) = K * wp^2/wz^2 * (s^2 + 2*Dz*wz*s + wz^2)/(s^2 + 2*Dp*wp*s + wp^2)
IIR_Filter::IIR_Filter(float k, float Tz, float Tp, float Ts, int typ)
{
    setup(k, Tz, Tp, Ts, typ);
}
IIR_Filter::IIR_Filter(float *b, float *a, int nb, int na, float yMin, float yMax)
{
    setup(b, a, nb, na);
    setLimits(yMin, yMax);
}

IIR_Filter::IIR_Filter(float *b, float *a, int nb, int na)
{
    setup(b, a, nb, na);
}

IIR_Filter::~IIR_Filter() {}

// G(s) = s/(T*s + 1)
void IIR_Filter::setup(float T, float Ts)
{
    /* filter orders */
    nb = 1;
    na = 1;

    /* filter coefficients */
    B = (float *)malloc((nb + 1) * sizeof(float));
    A = (float *)malloc(na * sizeof(float));
    double T_d = static_cast<double>(T);
    double Ts_d = static_cast<double>(Ts);
    B[0] = static_cast<float>(2.0 / (2.0 * T_d + Ts_d));
    B[1] = -B[0];
    A[0] = static_cast<float>(-(2.0 * T_d - Ts_d) / (2.0 * T_d + Ts_d));

    /* signal arrays */
    uk = (float *)malloc(nb * sizeof(float));
    yk = (float *)malloc(na * sizeof(float));
    uk[0] = 0.0f;
    yk[0] = 0.0f;

    /* dc-gain */
    this->K = 0.0f;

    /* min. and max. filter output */
    setLimits();
}

// G(s) = K/(T*s + 1)
void IIR_Filter::setup(float T, float Ts, float K)
{
    /* filter orders */
    nb = 1;
    na = 1;

    /* filter coefficients */
    B = (float *)malloc((nb + 1) * sizeof(float));
    A = (float *)malloc(na * sizeof(float));
    double T_d = static_cast<double>(T);
    double Ts_d = static_cast<double>(Ts);
    double K_d = static_cast<double>(K);
    B[0] = static_cast<float>(K_d * Ts_d / (Ts_d + 2.0 * T_d));
    B[1] = B[0];
    A[0] = static_cast<float>((Ts_d - 2.0 * T_d) / (Ts_d + 2.0 * T_d));

    /* signal arrays */
    uk = (float *)malloc(nb * sizeof(float));
    yk = (float *)malloc(na * sizeof(float));
    uk[0] = 0.0f;
    yk[0] = 0.0f;

    /* dc-gain */
    this->K = K;

    /* min. and max. filter output */
    setLimits();
}

// G(s) = K*w0^2/(s^2 + 2*D*w0*s + w0^2)
void IIR_Filter::setup(float w0, float D, float Ts, float K)
{
    /* filter orders */
    nb = 2;
    na = 2;

    /* filter coefficients */
    B = (float *)malloc((nb + 1) * sizeof(float));
    A = (float *)malloc(na * sizeof(float));
    double w0_d = static_cast<double>(w0);
    double D_d = static_cast<double>(D);
    double Ts_d = static_cast<double>(Ts);
    double K_d = static_cast<double>(K);
    double k0 = Ts_d * Ts_d * w0_d * w0_d;
    double k1 = 4.0 * D_d * Ts_d * w0_d;
    double k2 = k0 + k1 + 4.0;
    B[0] = static_cast<float>(K_d * k0 / k2);
    B[1] = static_cast<float>(2.0 * K_d * k0 / k2);
    B[2] = B[0];
    A[0] = static_cast<float>((2.0 * k0 - 8.0) / k2);
    A[1] = static_cast<float>((k0 - k1 + 4.0) / k2);

    /* signal arrays */
    uk = (float *)malloc(nb * sizeof(float));
    yk = (float *)malloc(na * sizeof(float));
    uk[0] = uk[1] = 0.0f;
    yk[0] = yk[1] = 0.0f;

    /* dc-gain */
    this->K = K;

    /* min. and max. filter output */
    setLimits();
}
// G(s) = K*w0^2/(s^2 + 2*D*w0*s + w0^2)
void IIR_Filter::setup(float k, float Tz, float Tp, float Ts, int typ)
{
    /* filter orders */
    nb = 1;
    na = 1;

    /* filter coefficients */
    B = (float *)malloc((nb + 1) * sizeof(float));
    A = (float *)malloc(na * sizeof(float));
    double Tz_d = static_cast<double>(Tz);
    double Tp_d = static_cast<double>(Tp);
    double Ts_d = static_cast<double>(Ts);
    // bilinear trafo
    double b1 = (2.0 * Tz_d + Ts_d) / (2.0 * Tp_d + Ts_d);
    double b0 = (Ts_d - 2 * Tz_d) / (2.0 * Tp_d + Ts_d);
    double a0 = (Ts_d - 2 * Tp_d) / (2.0 * Tp_d + Ts_d);
    B[0] = static_cast<float>(b1);
    B[1] = static_cast<float>(b0);
    A[0] = static_cast<float>(a0);

    /* signal arrays */
    uk = (float *)malloc(nb * sizeof(float));
    yk = (float *)malloc(na * sizeof(float));
    uk[0] = 0.0f;
    yk[0] = 0.0f;

    /* dc-gain */
    this->K = k;

    /* min. and max. filter output */
    setLimits();
}

// G(s) = K * wp^2/wz^2 * (s^2 + 2*Dz*wz*s + wz^2)/(s^2 + 2*Dp*wp*s + wp^2)
void IIR_Filter::setup(float wz, float Dz, float wp, float Dp, float Ts, float K)
{
    /* filter orders */
    nb = 2;
    na = 2;

    /* filter coefficients */
    B = (float *)malloc((nb + 1) * sizeof(float));
    A = (float *)malloc(na * sizeof(float));
    double wz_d = static_cast<double>(wz);
    double Dz_d = static_cast<double>(Dz);
    double wp_d = static_cast<double>(wp);
    double Dp_d = static_cast<double>(Dp);
    double Ts_d = static_cast<double>(Ts);
    double K_d = static_cast<double>(K);
    B[0] = static_cast<float>(K_d * (wp_d * wp_d * (Ts_d * Ts_d * wz_d * wz_d + 4.0 * Dz_d * Ts_d * wz_d + 4.0)) / (wz_d * wz_d * (Ts_d * Ts_d * wp_d * wp_d + 4.0 * Dp_d * Ts_d * wp_d + 4.0)));
    B[1] = static_cast<float>(K_d * (2.0 * wp_d * wp_d * (Ts_d * Ts_d * wz_d * wz_d - 4.0)) / (wz_d * wz_d * (Ts_d * Ts_d * wp_d * wp_d + 4.0 * Dp_d * Ts_d * wp_d + 4.0)));
    B[2] = static_cast<float>(K_d * (wp_d * wp_d * (Ts_d * Ts_d * wz_d * wz_d - 4.0 * Dz_d * Ts_d * wz_d + 4.0)) / (wz_d * wz_d * (Ts_d * Ts_d * wp_d * wp_d + 4.0 * Dp_d * Ts_d * wp_d + 4.0)));
    A[0] = static_cast<float>((2.0 * Ts_d * Ts_d * wp_d * wp_d - 8.0) / (Ts_d * Ts_d * wp_d * wp_d + 4.0 * Dp_d * Ts_d * wp_d + 4.0));
    A[1] = static_cast<float>((Ts_d * Ts_d * wp_d * wp_d - 4.0 * Dp_d * Ts_d * wp_d + 4.0) / (Ts_d * Ts_d * wp_d * wp_d + 4.0 * Dp_d * Ts_d * wp_d + 4.0));

    /* signal arrays */
    uk = (float *)malloc(nb * sizeof(float));
    yk = (float *)malloc(na * sizeof(float));
    uk[0] = uk[1] = 0.0f;
    yk[0] = yk[1] = 0.0f;

    /* dc-gain */
    this->K = K;

    /* min. and max. filter output */
    setLimits();
}

/*
    // Matlab Code: G(s)
    G = c2d(G, Ts, 'tustin');
    set(G, 'variable', 'z^-1');
    G.num{1}
    G.den{1}
    ans =
       6.279838916529264  12.263724222078224   6.087828740556124
    ans =
       1.000000000000000   0.522438051674467   0.857406486617799
    // C++ code:
    int nb = 2;
    int na = 2;
    float B[nb+1] = {6.279838916529264,  12.263724222078224,   6.087828740556124};
    float A[na+1] = {1.000000000000000,   0.522438051674467,   0.857406486617799};
    IIR_Filter G(B, A, nb, na);
*/

void IIR_Filter::setup(float *b, float *a, int nb, int na, float yMin, float yMax)
{
    setup(b, a, nb, na);
    setLimits(yMin, yMax);
}

void IIR_Filter::setup(float *b, float *a, int nb, int na)
{
    /* filter orders */
    this->nb = nb;
    this->na = na;

    /* filter coefficients and signal arrays */
    B = (float *)malloc((nb + 1) * sizeof(float));
    A = (float *)malloc(na * sizeof(float));
    uk = (float *)malloc(nb * sizeof(float));
    yk = (float *)malloc(na * sizeof(float));
    B[0] = b[0];
    float sum_B = b[0];
    for (int k = 0; k < nb; k++)
    {
        B[k + 1] = b[k + 1];
        uk[k] = 0.0f;
        sum_B += b[k + 1];
    }
    float sum_A = 1.0f;
    for (int k = 0; k < na; k++)
    {
        A[k] = a[k + 1];
        yk[k] = 0.0f;
        sum_A += a[k + 1];
    }

    /* dc-gain */
    if (fabs(sum_A) >= 0.0000001f)
    {
        this->K = sum_B / sum_A;
    }
    else
    {
        this->K = 1.0f;
    }

    /* min. and max. filter output */
    setLimits();
}

void IIR_Filter::reset()
{
    reset(0.0f, K * 0.0f);
}

void IIR_Filter::reset(float u)
{
    reset(u, K * u);
}

void IIR_Filter::reset(float u, float y)
{
    for (unsigned k = 0; k < nb; k++)
        uk[k] = u;
    for (unsigned k = 0; k < na; k++)
        yk[k] = y;
}

void IIR_Filter::setLimits(float yMax)
{
    setLimits(-yMax, yMax);
}

void IIR_Filter::setLimits(float yMin, float yMax)
{
    this->yMin = yMin;
    this->yMax = yMax;
}

float IIR_Filter::get_output()
{
    return yk[0];
}

float IIR_Filter::prewarp(float T, float Ts)
{
    double T_d = static_cast<double>(T);
    double Ts_d = static_cast<double>(Ts);
    return static_cast<float>(Ts_d / (2.0 * tan(Ts_d / (2.0 * T_d))));
}

// void IIR_Filter::print_filter_coeff()
// {
//     printf("B = [");
//     for (uint8_t k = 0; k <= nb; k++)
//     {
//         printf("%6.6e", B[k]);
//         if (k < nb)
//             printf(",");
//     }
//     printf("]\r\n");
//     printf("A = [1.0, ");
//     for (uint8_t k = 0; k < na; k++)
//     {
//         printf("%6.6e", A[k]);
//         if (k < na - 1)
//             printf(",");
//     }
//     printf("]\r\n");
// }

/*
    the filter step is performed as follows:
    (b_0 + b_1*z^-1 + ... + b_nb*z^-nb) * U(z) = (1 + a_1*z^-1 + ... + a_na*z^-na)) * Y(z)
    y(k) =   b_0*u(k)   + b_1*u(k-1) + ... + b_nb*u(k-nb) + ...
           - a_1*y(k-1) - a_2*y(k-2) - ... - a_na*y(k-na)
    but stored is only
    B = ( B[0], B[1],   ... , B[nb]   ) := ( b_0, b_1, ... , b_nb )
    A =       ( A[0],   ... , A[na-1] ) :=      ( a_1, ... , a_na )
    u =       ( u[k-1], ... , u[k-nb] )
    y =       ( y[k-1], ... , y[k-na] )
*/
float IIR_Filter::filter(float u)
{
    /* perform filter step */
    float y = B[0] * u;
    for (unsigned k = 0; k < nb; k++)
        y += B[k + 1] * uk[k];
    for (unsigned k = 0; k < na; k++)
        y -= A[k] * yk[k];

    /* update and saturate storage */
    for (unsigned k = nb - 1; k > 0; k--)
        uk[k] = uk[k - 1];
    uk[0] = u;
    for (unsigned k = na - 1; k > 0; k--)
        yk[k] = yk[k - 1];
    yk[0] = saturate(y, yMin, yMax);

    return yk[0];
}

float IIR_Filter::saturate(float y, float yMin, float yMax)
{
    if (y > yMax)
    {
        y = yMax;
    }
    else if (y < yMin)
    {
        y = yMin;
    }
    return y;
}