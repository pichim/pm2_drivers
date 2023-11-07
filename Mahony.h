#ifndef MAHONY_H_
#define MAHONY_H_

#include <mbed.h>

#include "Param.h"

class Mahony
{
public:
    Mahony();
    Mahony(float kp, float ki, float Ts);
    virtual ~Mahony();

    void Setup(float kp, float ki, float Ts);
    void Update(Eigen::Vector3f gyro, Eigen::Vector3f acc);
    void Update(Eigen::Vector3f gyro, Eigen::Vector3f acc, Eigen::Vector3f mag);
    Eigen::Quaternionf GetOrientationAsQuaternion();
    Eigen::Vector3f GetOrientationAsRPYAngles();
    float GetTiltAngle();

private:
    float m_kp = 0.0f;
    float m_ki = 0.0f;
    float m_Ts = 1.0f;
    Eigen::Quaternionf m_quat;
    Eigen::Vector3f m_bias;
    Eigen::Vector3f m_rpy;
    float m_tilt = 0.0f;

    void initialise();
    void updateOrientation(Eigen::Vector3f gyro, Eigen::Vector3f e);
    Eigen::Vector3f quat2rpy(Eigen::Quaternionf quat);
    Eigen::Vector3f calcRotationError(Eigen::Vector3f v1, Eigen::Vector3f v2);
};

#endif /* MAHONY_H_ */
