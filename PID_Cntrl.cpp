#include "PID_Cntrl.h"

/*
                    Ts
    C(z) = (P + I ---------- + D * tustin(s / (tau_f * s + 1))) * tustin(1 / (tau_ro * s + 1))
                 1 - z^-1
    - Corresponds to MATLAB command: C = pid(P, I, D, tau_f, Ts, 'IFormula', 'BackwardEuler', 'DFormula', 'Trapezoidal') * c2d(tf(1, [tau_ro 1]), Ts, 'tustin')
    - Anti-Windup: Saturation of Ipart and u = P*e + IPart + DPart
*/

PID_Cntrl::PID_Cntrl(float I, float Ts, float uMin, float uMax)
{
    setup(I, Ts, uMin, uMax);
}

PID_Cntrl::PID_Cntrl(float P, float I, float Ts, float uMin, float uMax)
{
    setup(P, I, Ts, uMin, uMax);
}

PID_Cntrl::PID_Cntrl(float P, float I, float D, float Ts, float uMin, float uMax)
{
    setup(P, I, D, Ts, uMin, uMax);
}

PID_Cntrl::PID_Cntrl(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax)
{
    setup(P, I, D, tau_f, Ts, uMin, uMax);
}

PID_Cntrl::PID_Cntrl(float P, float I, float D, float tau_f, float tau_ro, float Ts, float uMin, float uMax)
{
    setup(P, I, D, tau_f, tau_ro, Ts, uMin, uMax);
}

PID_Cntrl::~PID_Cntrl() {}

void PID_Cntrl::reset(float initValue)
{
    IPart = initValue;
    Dpart = 0.0f;
    d_old = 0.0f;
    u_old = initValue;
    uf = initValue;
}

void PID_Cntrl::setup(float I, float Ts, float uMin, float uMax)
{
    P = 0.0f;
    D = 0.0f;
    double Ts_d = static_cast<double>(Ts);
    tau_f = static_cast<float>(Ts_d / (0.95 * M_PI));
    tau_ro = 0.0f;
    setup(P, I, D, tau_f, tau_ro, Ts, uMin, uMax);
}

void PID_Cntrl::setup(float P, float I, float Ts, float uMin, float uMax)
{
    D = 0.0f;
    double Ts_d = static_cast<double>(Ts);
    tau_f = static_cast<float>(Ts_d / (0.95 * M_PI));
    tau_ro = 0.0f;
    setup(P, I, D, tau_f, tau_ro, Ts, uMin, uMax);
}

void PID_Cntrl::setup(float P, float I, float D, float Ts, float uMin, float uMax)
{
    double Ts_d = static_cast<double>(Ts);
    tau_f = static_cast<float>(Ts_d / (0.95 * M_PI));
    tau_ro = 0.0f;
    setup(P, I, D, tau_f, tau_ro, Ts, uMin, uMax);
}

void PID_Cntrl::setup(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax)
{
    tau_ro = 0.0f;
    setup(P, I, D, tau_f, tau_ro, Ts, uMin, uMax);
}

void PID_Cntrl::setup(float P, float I, float D, float tau_f, float tau_ro, float Ts, float uMin, float uMax)
{
    setCoefficients(P, I, D, tau_f, tau_ro, Ts);
    setLimits(uMin, uMax);
    reset();
}

void PID_Cntrl::setCoeff_P(float P)
{
    this->P = P;
}

void PID_Cntrl::setCoeff_I(float I)
{
    this->I = I;
    updateCoeff_I(I, Ts);
}

void PID_Cntrl::setCoeff_D(float D)
{
    this->D = D;
    updateCoeff_D(D, Ts, tau_f);
}

void PID_Cntrl::setCoeff_F(float F)
{
    this->F = F;
}

void PID_Cntrl::scale_PIDT2_param(float scale)
{
    P = P_init * scale;
    I = I_init * scale;
    D = D_init * scale;
    updateCoeff_I(I, Ts);
    updateCoeff_D(D, Ts, tau_f);
}

float PID_Cntrl::update(float e)
{
    if (bi != 0)
        IPart = saturate(IPart + bi * e, uIMin, uIMax);
    else
        IPart = 0.0f;
    Dpart = bd * (e - d_old) - ad * Dpart;
    d_old = e;
    float u = P * e + IPart + Dpart;
    uf = saturate(bf * (u + u_old) - af * uf, uMin, uMax);
    u_old = u;
    return uf;
}

float PID_Cntrl::update(float e, float y)
{
    if (bi != 0)
        IPart = saturate(IPart + bi * e, uIMin, uIMax);
    else
        IPart = 0.0;
    Dpart = bd * (y - d_old) - ad * Dpart;
    d_old = y;
    float u = P * e + IPart - Dpart;
    uf = saturate(bf * (u + u_old) - af * uf, uMin, uMax);
    u_old = u;
    return uf;
}

float PID_Cntrl::update(float w, float y_p, float y_i, float y_d)
{
    if (bi != 0)
        IPart = saturate(IPart + bi * (w - y_i), uIMin, uIMax);
    else
        IPart = 0.0;
    Dpart = bd * (y_d - d_old) - ad * Dpart;
    d_old = y_d;
    float u = P * (w - y_p) + IPart - Dpart + F * w;
    uf = saturate(bf * (u + u_old) - af * uf, uMin, uMax);
    u_old = u;
    return uf;
}

void PID_Cntrl::setLimits(float uMin, float uMax)
{
    this->uMin = uMin;
    this->uMax = uMax;
    this->uIMin = uMin;
    this->uIMax = uMax;
}

void PID_Cntrl::setIntegratorLimits(float uIMin, float uIMax)
{
    this->uIMin = uIMin;
    this->uIMax = uIMax;
}

float PID_Cntrl::prewarp(float T, float Ts)
{
    double T_d = static_cast<double>(T);
    double Ts_d = static_cast<double>(Ts);
    return static_cast<float>(Ts_d / (2.0 * tan(Ts_d / (2.0 * T_d))));
}

float PID_Cntrl::get_ulimit()
{
    return uMax;
}

float PID_Cntrl::get_P_gain()
{
    return P;
}

float PID_Cntrl::get_bd()
{
    return bd;
}

float PID_Cntrl::get_ad()
{
    return ad;
}

float PID_Cntrl::get_current_output(void)
{
    return uf;
}

void PID_Cntrl::setCoefficients(float P, float I, float D, float tau_f, float tau_ro, float Ts)
{
    /* store parameters */
    this->P = P;
    this->I = I;
    this->D = D;
    this->tau_f = tau_f;
    this->tau_ro = tau_ro;
    this->Ts = Ts;
    updateCoeff_I(I, Ts);
    updateCoeff_D(D, Ts, tau_f);
    updateCoeff_RO(Ts, tau_ro);

    /* store initial parameters */
    this->P_init = P;
    this->I_init = I;
    this->D_init = D;
}

void PID_Cntrl::updateCoeff_I(float I, float Ts)
{
    double I_d = static_cast<double>(I);
    double Ts_d = static_cast<double>(Ts);
    bi = static_cast<float>(I_d * Ts_d);
}

void PID_Cntrl::updateCoeff_D(float D, float Ts, float tau_f)
{
    double D_d = static_cast<double>(D);
    double Ts_d = static_cast<double>(Ts);
    double tau_f_d = static_cast<double>(tau_f);
    bd = static_cast<float>(2.0 * D_d / (Ts_d + 2.0 * tau_f_d));
    ad = static_cast<float>((Ts_d - 2.0 * tau_f_d) / (Ts_d + 2.0 * tau_f_d));
}

void PID_Cntrl::updateCoeff_RO(float Ts, float tau_ro)
{
    double Ts_d = static_cast<double>(Ts);
    double tau_ro_d = static_cast<double>(tau_ro);
    bf = static_cast<float>(Ts_d / (Ts_d + 2.0 * tau_ro_d));
    af = static_cast<float>((Ts_d - 2.0 * tau_ro_d) / (Ts_d + 2.0 * tau_ro_d));
}

float PID_Cntrl::saturate(float u, float uMin, float uMax)
{
    return (u > uMax) ? uMax : (u < uMin) ? uMin : u;
}