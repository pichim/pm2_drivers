#ifndef MAHONY_H_
#define MAHONY_H_

#include "eigen/Dense.h"

class Mahony
{
public:
    explicit Mahony();
    explicit Mahony(float kp, float ki, float Ts);
    virtual ~Mahony();

    void setup(float kp, float ki, float Ts);
    // % bessel
    // p = 2;         % pole at p rad/s
    // kp = 2 * p;
    // ki = kp^2 / 3;
    void setGains(float kp = 2.0f * 2.0f, float ki = 16.0f / 3.0f);
    void setSamplingTime(float Ts);
    void update(Eigen::Vector3f gyro, Eigen::Vector3f acc);
    void update(Eigen::Vector3f gyro, Eigen::Vector3f acc, Eigen::Vector3f mag);
    Eigen::Quaternionf getOrientationAsQuaternion() const;
    Eigen::Vector3f getOrientationAsRPYAngles() const;
    float getTiltAngle() const;

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
