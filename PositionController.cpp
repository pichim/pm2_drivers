#include "PositionController.h"

const float PositionController::TS = 0.001f;                       // period of 1 ms
const float PositionController::LOWPASS_FILTER_FREQUENCY = 100.0f; // given in [rad/s]
const float PositionController::MIN_DUTY_CYCLE = 0.01f;            // minimum duty-cycle
const float PositionController::MAX_DUTY_CYCLE = 0.99f;            // maximum duty-cycle

PositionController::PositionController(float counts_per_turn, float kn, float max_voltage, FastPWM& pwm, EncoderCounter& encoderCounter) : pwm(pwm), encoderCounter(encoderCounter), thread(osPriorityHigh, 4096)
{
    this->counts_per_turn = counts_per_turn;
    setFeedForwardGain(kn);
    setSpeedCntrlGain(0.1f);
    this->max_voltage = max_voltage;
    this->max_speed   = kn*max_voltage;
    setPositionCntrlGain(22.0f);

    // initialise pwm
    pwm.period(0.00005); // pwm period of 50 us
    pwm.write(0.5);      // duty-cycle of 50%

    // initialise
    previousValueCounter = encoderCounter.read();
    speedFilter.setPeriod(TS);
    speedFilter.setFrequency(LOWPASS_FILTER_FREQUENCY);
    desiredSpeed = 0.0f;
    actualSpeed = 0.0f;
    this->initialRotation = (float)encoderCounter.read()/counts_per_turn;
    actualRotation  = initialRotation;
    desiredRotation = initialRotation;
    
    motion.setProfileVelocity(max_voltage * kn);
    float maxAcceleration = 22.0f * max_voltage * kn * 0.4f; // pmic, 13.05.2022, only 40%, based on simple measurement, max_voltage * kn is gearratio
    motion.setProfileAcceleration(maxAcceleration);
    motion.setProfileDeceleration(maxAcceleration);

    // set up thread
    thread.start(callback(this, &PositionController::run));
    ticker.attach(callback(this, &PositionController::sendThreadFlag), std::chrono::microseconds{static_cast<long int>(1.0e6f * TS)});
}

PositionController::~PositionController()
{
    ticker.detach();
}

/**
 * Reads the speed in RPM (rotations per minute).
 * @return actual speed in RPM.
 */
float PositionController::getSpeedRPM()
{
    return actualSpeed;
}

/**
 * Reads the speed in RPS (rotations per second).
 * @return actual speed in RPS.
 */
float PositionController::getSpeedRPS()
{
    return actualSpeed/60.0f;
}

/**
 * Sets desired rotation (1 corresponds to 360 deg).
 */
void PositionController::setDesiredRotation(float desiredRotation)
{
    this->desiredRotation = initialRotation + desiredRotation;
}

/**
 * Reads the number of rotations (1 corresponds to 360 deg).
 * @return actual rotations.
 */
float PositionController::getRotation()
{
    return actualRotation - initialRotation;
}

/**
 * Sets the feed-forward gain.
 */
void PositionController::setFeedForwardGain(float kn)
{
    this->kn = kn;
}

/**
 * Sets the gain of the speed controller (p-controller).
 */
void PositionController::setSpeedCntrlGain(float kp)
{
    this->kp = kp;
}

/**
 * Sets the gain of the position controller (p-controller).
 */
void PositionController::setPositionCntrlGain(float p)
{
    this->p = p;
}

/**
 * Sets the maximum Velocity in RPS.
 */
void PositionController::setMaxVelocityRPS(float maxVelocityRPS)
{
    if (maxVelocityRPS*60.0f <= max_speed)
        motion.setProfileVelocity(maxVelocityRPS*60.0f);
    else
        motion.setProfileVelocity(max_speed);
}

/**
 * Sets the maximum Velocity in RPM.
 */
void PositionController::setMaxVelocityRPM(float maxVelocityRPM)
{
    if (maxVelocityRPM <= max_speed)
        motion.setProfileVelocity(maxVelocityRPM);
    else
        motion.setProfileVelocity(max_speed);
}

/**
 * Sets the maximum Acceleration in RPS/sec.
 */
void PositionController::setMaxAccelerationRPS(float maxAccelerationRPS)
{
    motion.setProfileAcceleration(maxAccelerationRPS*60.0f);
    motion.setProfileDeceleration(maxAccelerationRPS*60.0f);
}

/**
 * Sets the maximum Acceleration in RPM/sec.
 */
void PositionController::setMaxAccelerationRPM(float maxAccelerationRPM)
{
    motion.setProfileAcceleration(maxAccelerationRPM);
    motion.setProfileDeceleration(maxAccelerationRPM);
}

void PositionController::run()
{
    while(true) {
        // wait for the periodic signal
        ThisThread::flags_wait_any(threadFlag);

        // calculate actual speed of motors in [rpm]
        short valueCounter = encoderCounter.read();
        short countsInPastPeriod = valueCounter - previousValueCounter;
        previousValueCounter = valueCounter;
        actualSpeed = speedFilter.filter((float)countsInPastPeriod/counts_per_turn/TS*60.0f);
        actualRotation = actualRotation + actualSpeed/60.0f*TS;

        // trajectory needs to be calculated in rotations*60 so that all units are in RPM
        motion.incrementToPosition(60.0f * desiredRotation, TS);       
        float desiredRotationMotion = motion.getPosition();
        float desiredVelocityMotion = motion.getVelocity();

        // calculate motor phase voltages
        desiredSpeed  = p *(desiredRotationMotion - 60.0f * actualRotation) + 0.0f * desiredVelocityMotion; // using velocity feedforward here will lead to overshoot
        
        if (desiredSpeed < -max_speed) desiredSpeed = -max_speed;
        else if (desiredSpeed > max_speed) desiredSpeed = max_speed;
        
        float voltage = kp*(desiredSpeed - actualSpeed) + desiredSpeed/kn;
        // calculate, limit and set duty cycles
        float dutyCycle = 0.5f + 0.5f*voltage/max_voltage;
        if (dutyCycle < MIN_DUTY_CYCLE) dutyCycle = MIN_DUTY_CYCLE;
        else if (dutyCycle > MAX_DUTY_CYCLE) dutyCycle = MAX_DUTY_CYCLE;
        pwm.write(static_cast<double>(dutyCycle));
    }
}

void PositionController::sendThreadFlag()
{
    thread.flags_set(threadFlag);
}