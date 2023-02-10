#ifndef SpeedController_H_
#define SpeedController_H_
#include <cstdlib>
#include <mbed.h>
#include "EncoderCounter.h"
#include "LowpassFilter.h"
#include "ThreadFlag.h"
#include "FastPWM.h"
#include "Motion.h"

class SpeedController
{
public:

    SpeedController(float counts_per_turn, float kn, float max_voltage, FastPWM& pwm, EncoderCounter& encoderCounter);

    virtual ~SpeedController();

    void     setDesiredSpeedRPS(float desiredSpeed);
    float    getSpeedRPM();
    float    getSpeedRPS();
    void     setFeedForwardGain(float kn);
    void     setSpeedCntrlGain(float kp);
    void     setMaxVelocityRPS(float maxVelocityRPS);
    void     setMaxVelocityRPM(float maxVelocityRPM);
    void     setMaxAccelerationRPS(float maxAccelerationRPS);
    void     setMaxAccelerationRPM(float maxAccelerationRPM);

private:

    void     setDesiredSpeedRPM(float desiredSpeed);

    static const float    TS;
    static const float    LOWPASS_FILTER_FREQUENCY;
    static const float    MIN_DUTY_CYCLE;
    static const float    MAX_DUTY_CYCLE;
    static const uint16_t DEFAULT_PERIOD_MUS;

    float counts_per_turn;
    float kn;
    float kp;
    float max_voltage;

    FastPWM&           pwm;
    EncoderCounter&    encoderCounter;
    short              previousValueCounter;
    LowpassFilter      speedFilter;
    float              desiredSpeed;
    float              actualSpeed;
    // float              actualAngle;
    
    Motion             motion;

    ThreadFlag         threadFlag;
    Thread             thread;
    Ticker             ticker;

    void               run();
    void               sendThreadFlag();
};

#endif /* SpeedController_H_ */