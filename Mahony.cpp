/**
 * notes:
 * - roll, pitch and therefor tilt can be estimated without magnetometer
 * - yaw can only be estimated with magnetometer
 * - magnetic north is not the same as true north, taking magnetic declination into account and implement it would be necessary for true north reference
 * - proper calibration of the magnetometer is crucial for yaw estimation
  */

#include "Mahony.h"

Mahony::Mahony()
{
    initialise();
}

Mahony::Mahony(float kp, float ki, float Ts)
{
    initialise();
    setup(kp, ki, Ts);
}

Mahony::~Mahony()
{
    
}

void Mahony::update(Eigen::Vector3f gyro, Eigen::Vector3f acc)
{
    Eigen::Vector3f g_n(                                          2.0f * ( m_quat.x()*m_quat.z() - m_quat.w()*m_quat.y() ),
                                                                  2.0f * ( m_quat.y()*m_quat.z() + m_quat.w()*m_quat.x() ),
                         ( m_quat.w()*m_quat.w() - m_quat.x()*m_quat.x() - m_quat.y()*m_quat.y() + m_quat.z()*m_quat.z() )  );
    // Eigen::Vector3f e = acc.normalized().cross( g_n.normalized() );
    Eigen::Vector3f e = calcRotationError(acc, g_n);

    updateOrientation(gyro, e);
}

void Mahony::update(Eigen::Vector3f gyro, Eigen::Vector3f acc, Eigen::Vector3f mag)
{
    Eigen::Matrix3f R = m_quat.toRotationMatrix();

    Eigen::Vector3f g_n = R.block<1,3>(2,0).transpose();
    // Eigen::Vector3f e = acc.normalized().cross( g_n.normalized() );
    Eigen::Vector3f e = calcRotationError(acc, g_n);

    Eigen::Vector3f h = R * mag.normalized();
    h(2) = 0.0f;
    Eigen::Vector3f b(h.norm(), 0.0f, 0.0f);
    // e += R.transpose() * h.cross(b);
    e += R.transpose() * calcRotationError(h, b) * h.norm();

    updateOrientation(gyro, e);
}

Eigen::Quaternionf Mahony::getOrientationAsQuaternion() const
{
    return m_quat;
}

Eigen::Vector3f Mahony::getOrientationAsRPYAngles() const
{
    return m_rpy;
}

float Mahony::getTiltAngle() const
{
    return m_tilt;
}

void Mahony::setup(float kp, float ki, float Ts)
{
    setGains(kp, ki);
    setSamplingTime(Ts);
}

void Mahony::setGains(float kp, float ki)
{
    m_kp = kp;
    m_ki = ki;
}

void Mahony::setSamplingTime(float Ts)
{
    m_Ts = Ts;
}

void Mahony::initialise()
{
    m_quat.setIdentity();
    m_bias.setZero();
    m_rpy.setZero();
}

Eigen::Vector3f Mahony::quat2rpy(Eigen::Quaternionf quat)
{
	// ||quat|| = 1
	Eigen::Vector3f rpy;
	// roll
	rpy(0) = atan2f( quat.y() * quat.z() + quat.w() * quat.x(), 0.5f - ( quat.x() * quat.x() + quat.y() * quat.y() ) );
	// pitch
	float sinarg = -2.0f * ( quat.x() * quat.z() - quat.w() * quat.y() );
	if (sinarg > 1.0f)
		sinarg = 1.0f;
	if (sinarg < -1.0f)
		sinarg = -1.0f;
	rpy(1) = asinf( sinarg );
	// yaw
	rpy(2) = atan2f( quat.x() * quat.y() + quat.w() * quat.z(), 0.5f - ( quat.y() * quat.y() + quat.z() * quat.z() ) );

	return rpy;
}

Eigen::Vector3f Mahony::calcRotationError(Eigen::Vector3f v1, Eigen::Vector3f v2)
{
    // https://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane
    Eigen::Vector3f vn = v1.cross(v2);
    float vn_norm = vn.norm();
    if (vn_norm != 0.0f) {
        vn /= vn_norm;
    }
    float ang = atan2f(v1.cross(v2).dot(vn), v1.dot(v2));
    return ang * vn;
}

void Mahony::updateOrientation(Eigen::Vector3f gyro, Eigen::Vector3f e)
{
    m_bias += m_ki * e * m_Ts;
    Eigen::Matrix<float, 4, 3> Q;
    Q << -m_quat.x(), -m_quat.y(), -m_quat.z(),
          m_quat.w(), -m_quat.z(),  m_quat.y(),
          m_quat.z(),  m_quat.w(), -m_quat.x(),
         -m_quat.y(),  m_quat.x(),  m_quat.w();

    // carefull here, Eigen Quaternions have the internal storage order [x y z w] but you inilialise them with quat(w, x, y, z)
    // so I rather type the following explicitly
    Eigen::Vector4f dquat = m_Ts * 0.5f * Q * ( gyro + m_bias + m_kp * e );
    m_quat.w() += dquat(0);
    m_quat.x() += dquat(1);
    m_quat.y() += dquat(2);
    m_quat.z() += dquat(3);
    m_quat.normalize();

    m_rpy = quat2rpy(m_quat);

    m_tilt = acosf( m_quat.w() * m_quat.w() - m_quat.x() * m_quat.x() - m_quat.y() * m_quat.y() + m_quat.z() * m_quat.z() );
}