#include "LineFollower.h"

// Constructor
LineFollower::LineFollower(PinName sda_pin,
                           PinName scl_pin,
                           float bar_dist,
                           float d_wheel,
                           float L_wheel,
                           float max_motor_vel_rps) : m_i2c(sda_pin, scl_pin),
                                                      m_SensorBar(m_i2c, bar_dist),
                                                      m_Thread()
{
    // geometric parameters
    m_r_wheel = d_wheel / 2.0f;
    m_L_wheel = L_wheel;

    // transform robot to wheel
    m_Cwheel2robot << m_r_wheel / 2.0f,       m_r_wheel / 2.0f,
                      m_r_wheel / m_L_wheel, -m_r_wheel / m_L_wheel;

    m_robot_coord.setZero();
    m_wheel_speed.setZero();

    m_max_motor_vel = max_motor_vel_rps;
    m_max_wheel_vel = max_motor_vel_rps;

    if (m_r_wheel != 0.0f)
        m_rotation_to_wheel_vel = m_L_wheel / (2.0f * m_r_wheel);

    // start thread
    m_Thread.start(callback(this, &LineFollower::followLine));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &LineFollower::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

// Deconstructor
LineFollower::~LineFollower()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

void LineFollower::setRotationalVelocityGain(float Kp, float Kp_nl)
{
    m_Kp = Kp;
    m_Kp_nl = Kp_nl;
}

void LineFollower::setMaxWheelVelocity(float max_wheel_vel)
{
    if (m_max_motor_vel < max_wheel_vel) {
        m_max_wheel_vel = m_max_motor_vel;
    } else if (max_wheel_vel < 0.0f) {
        m_max_wheel_vel = 0.0f;
    } else {
        m_max_wheel_vel = max_wheel_vel;
    }
}

float LineFollower::getAngleRadians() const
{
    return m_sensor_bar_avgAngleRad;
}

float LineFollower::getAngleDegrees() const
{
    return m_sensor_bar_avgAngleRad * 180.0f / M_PI;
}

float LineFollower::getRotationalVelocity() const
{
    return m_robot_coord(1);
}

float LineFollower::getTranslationalVelocity() const
{
    return m_robot_coord(0);
}

float LineFollower::getRightWheelVelocity() const
{
    return m_right_wheel_velocity;
}

float LineFollower::getLeftWheelVelocity() const
{
    return m_left_wheel_velocity;
}

bool LineFollower::isLedActive() const
{
    return isAnyLedActive;
}

// Thread task
void LineFollower::followLine()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        if (m_SensorBar.isAnyLedActive()) {
            m_sensor_bar_avgAngleRad = m_SensorBar.getAvgAngleRad();
            isAnyLedActive = true;
        } else {
            isAnyLedActive = false;
        }
        m_max_wheel_vel_rads = m_max_wheel_vel * 2 * M_PI;
        m_robot_coord(1) = ang_cntrl_fcn(m_Kp, m_Kp_nl, m_sensor_bar_avgAngleRad);
        m_robot_coord(0) = vel_cntrl_v2_fcn(m_max_wheel_vel_rads, m_rotation_to_wheel_vel, m_robot_coord(1), m_Cwheel2robot);

        m_wheel_speed = m_Cwheel2robot.inverse() * m_robot_coord;

        m_right_wheel_velocity = m_wheel_speed(0) / (2.0f * M_PI);
        m_left_wheel_velocity = m_wheel_speed(1) / (2.0f * M_PI);
    }
}

float LineFollower::ang_cntrl_fcn(float Kp, float Kp_nl, float angle)
{
    float retval = 0.0f;
    if (angle > 0.0f) {
        retval = Kp * angle + Kp_nl * angle * angle;
    } else if (angle <= 0.0f) {
        retval = Kp * angle - Kp_nl * angle * angle;
    }
    return retval;
}

float LineFollower::vel_cntrl_v2_fcn(float wheel_speed_max, float b, float robot_omega, Eigen::Matrix2f Cwheel2robot)
{
    Eigen::Matrix<float, 2, 1> wheel_speed;
    if (robot_omega > 0.0f) {
        wheel_speed(0) = wheel_speed_max;
        wheel_speed(1) = wheel_speed_max - 2.0f * b * robot_omega;
    } else {
        wheel_speed(0) = wheel_speed_max + 2.0f * b * robot_omega;
        wheel_speed(1) = wheel_speed_max;
    }
    Eigen::Matrix<float, 2, 1> robot_coord = Cwheel2robot * wheel_speed;

    return robot_coord(0);
}

void LineFollower::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
