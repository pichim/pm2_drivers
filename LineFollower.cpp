#include "LineFollower.h"
//Constructor
LineFollower::LineFollower(PinName pin_1,
                          PinName pin_2,
                          float bar_dist,
                          float d_wheel,
                          float L_wheel,
                          float max_motor_vel_rps
                          ) : m_i2c(pin_1, pin_2),
                              m_SensorBar(m_i2c, bar_dist),
                              m_Thread()
{
    //geometric parameters
    m_r_wheel = d_wheel / 2.0f;
    m_L_wheel = L_wheel;
    // transform robot to wheel
    m_Cwheel2robot <<  m_r_wheel / 2.0f   ,  m_r_wheel / 2.0f   ,
                     m_r_wheel / m_L_wheel, -m_r_wheel / m_L_wheel;

    m_robot_coord.setZero();
    m_wheel_speed.setZero();

    m_max_motor_vel = max_motor_vel_rps;
    m_max_wheel_vel = max_motor_vel_rps;
    
    m_b = m_L_wheel / (2.0f * m_r_wheel);

    // start thread
    m_Thread.start(callback(this, &LineFollower::followLine));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &LineFollower::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

//Deconstructor
LineFollower::~LineFollower()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

void LineFollower::setRotationalVelGain(float Kp, float Kp_nl)
{
    m_Kp = Kp;
    m_Kp_nl = Kp_nl;
}

void LineFollower::setMaxWheelVel(float new_max_wheel_vel)
{
    if (m_max_motor_vel < new_max_wheel_vel) {
        m_max_wheel_vel = m_max_motor_vel;
    } else if (new_max_wheel_vel < 0.0f) {
        m_max_wheel_vel = 0.001f;
    } else {
        m_max_wheel_vel = new_max_wheel_vel;
    }
}

float LineFollower::getAngRad()
{
    return m_sensor_bar_avgAngleRad;
}

float LineFollower::getRotationalVelocity()
{
    return m_robot_coord(1);
}

float LineFollower::getTranslationalVelocity()
{
    return m_robot_coord(0);
}

float LineFollower::getRightWheelVelocity()
{
    return m_right_wheel_velocity;
}

float LineFollower::getLeftWheelVelocity()
{
    return m_left_wheel_velocity;
}

bool LineFollower::isLedActive()
{
    return isAnyLedActive; 
}

//Thread task
void LineFollower::followLine()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        // TODO - THINK HOW SHOULD IT BE EVALUETED
        if (m_SensorBar.isAnyLedActive()) {
            m_sensor_bar_avgAngleRad = m_SensorBar.getAvgAngleRad();
            isAnyLedActive = true;
        } else {
            isAnyLedActive = false;
        }
        m_max_wheel_vel_rads = m_max_wheel_vel * 2 * M_PI;
        m_robot_coord(1) = ang_cntrl_fcn(m_Kp, m_Kp_nl, m_sensor_bar_avgAngleRad);
        m_robot_coord(0) = vel_cntrl_v2_fcn(m_max_wheel_vel_rads, m_b, m_robot_coord(1), m_Cwheel2robot);

        m_wheel_speed = m_Cwheel2robot.inverse() * m_robot_coord;

        // Verify sides
        m_right_wheel_velocity = m_wheel_speed(0) / (2.0f * M_PI);
        m_left_wheel_velocity = m_wheel_speed(1) / (2.0f * M_PI);

    }

}

float LineFollower::ang_cntrl_fcn(const float& Kp, const float& Kp_nl, const float& angle)
{
    static float retval = 0.0f;
    if (angle > 0) {
        retval = Kp * angle + Kp_nl * angle * angle;
    } else if (angle <= 0) {
        retval = Kp * angle - Kp_nl * angle * angle;
    }
    return retval;
}

float LineFollower::vel_cntrl_v2_fcn(const float& wheel_speed_max, const float& b, const float& robot_omega, const Eigen::Matrix2f& Cwheel2robot)
{
    static Eigen::Matrix<float, 2, 2> _wheel_speed;
    static Eigen::Matrix<float, 2, 2> _robot_coord;
    if (robot_omega > 0) {
        _wheel_speed(0) = wheel_speed_max;
        _wheel_speed(1) = wheel_speed_max - 2*b*robot_omega;
    } else {
        _wheel_speed(0) = wheel_speed_max + 2*b*robot_omega;
        _wheel_speed(1) = wheel_speed_max;
    }
    _robot_coord = Cwheel2robot * _wheel_speed;

    return _robot_coord(0);
}

void LineFollower::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}

