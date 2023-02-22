/* PM2_Libary */
#include "EncoderCounter.h"
#include "Servo.h"
#include "SpeedController.h"
#include "PositionController.h"
#include "FastPWM.h"
#include "RangeFinder.h"
// #include "LSM9DS1_i2c.h" // this thing is a mess
#include "SensorBar.h"
#include "DebounceIn.h"

/** PES-Board Versions
 *
 * New Version                             |  Old Version
 *
 * DigitalOut enable_motors(PB_15);        |  DigitalOut enable_motors(PB_2);
 * FastPWM pwm_M1(PB_13);                  |  FastPWM pwm_M1(PA_8);
 * FastPWM pwm_M2(PA_9);                   |  FastPWM pwm_M2(PA_9);
 * EncoderCounter  encoder_M1(PA_6, PC_7); |  EncoderCounter  encoder_M1(PB_6, PB_7);
 * EncoderCounter  encoder_M2(PB_6, PB_7); |  EncoderCounter  encoder_M2(PA_6, PC_7);
 */