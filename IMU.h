#ifndef IMU_H_
#define IMU_H_

#include "mbed.h"
#include "eigen/Dense.h"

#include "LSM9DS1/LSM9DS1.h"
#include "LinearCharacteristics3.h"
#include "Mahony.h"
#include "ThreadFlag.h"

#define IMU_THREAD_TS_MS 20
#define IMU_THREAD_PRIORITY osPriorityHigh
#define IMU_THREAD_SIZE 4096
#define IMU_PIN_SDA PC_9
#define IMU_PIN_SCL PA_8
#define IMU_DO_PRINTF false
#define IMU_DO_USE_STATIC_ACC_CALIBRATION true  // if this is false then acc gets averaged at the beginning and printed to the console
#define IMU_DO_USE_STATIC_MAG_CALIBRATION false // if this is false then no mag calibration gets applied, e.g. A_mag = I, b_mag = 0
#define IMU_THREAD_DO_USE_MAG_FOR_MAHONY_UPDATE false

namespace Parameters
{
    // % Bessel -> D = sqrt(3)/2
    // p = 2;    % pole at p rad/s
    // kp = 2 * p;
    // ki = kp^2 / 3;
    static const float kp = 2.0f * 2.0f;
    static const float ki = kp * kp / 3.0f;
    // mag_calibrated = A_mag * ( mag - b_mag )
    static const Eigen::Matrix3f A_mag = (Eigen::Matrix3f() << 1.0000000f, 0.0000000f, 0.0000000f,
                                                               0.0000000f, 1.0000000f, 0.0000000f,
                                                               0.0000000f, 0.0000000f, 1.0000000f).finished();
    static const Eigen::Vector3f b_mag = (Eigen::Vector3f() << 0.0000000f, 0.0000000f, 0.0000000f).finished();
    static const Eigen::Vector3f b_acc = (Eigen::Vector3f() << 0.0000000f, 0.0000000f, 0.0000000f).finished();
}

class ImuData
{
public:
    ImuData() {
        initialise();
    };
    virtual ~ImuData(){};

    Eigen::Vector3f gyro, acc, mag;
    Eigen::Quaternionf quat;
    Eigen::Vector3f rpy;
    float tilt = 0.0f;

    void initialise() {
        gyro.setZero();
        acc.setZero();
        mag.setZero();
        quat.setIdentity();
        rpy.setZero();
    };
};

class IMU
{
public:
    explicit IMU();
    virtual ~IMU();

    ImuData getImuData() const;

private:
    ImuData m_ImuData;
    LSM9DS1 m_ImuLSM9DS1;
    LinearCharacteristics3 m_magCalib;
    Mahony m_Mahony;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    void startThread();
    void threadTask();
    void sendThreadFlag();
};

#endif /* IMU_H_ */
