#include "LinearCharacteristics3.h"

LinearCharacteristics3::LinearCharacteristics3()
{
    m_A.setIdentity();
    m_b.setZero();
}

LinearCharacteristics3::~LinearCharacteristics3() {}

void LinearCharacteristics3::setCalibrationParameter(Eigen::Matrix3f A, Eigen::Vector3f b)
{
    m_A = A;
    m_b = b;
}

void LinearCharacteristics3::setLimits(float& lowerLimit, float& upperLimit)
{
    m_lowerLimit = lowerLimit;
    m_upperLimit = upperLimit;
}

Eigen::Vector3f LinearCharacteristics3::applyCalibration(Eigen::Vector3f& x)
{
    Eigen::Vector3f y = m_A * ( x - m_b );
    for(uint8_t i = 0; i < 3; i++) {
        if(y(i) < m_lowerLimit) y(i) = m_lowerLimit;
        if(y(i) > m_upperLimit) y(i) = m_upperLimit;
    }
    return y;
}
