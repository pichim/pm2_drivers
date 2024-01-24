
#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_

#include <mbed.h>
#include <math.h>

#include "pm2_drivers/SensorBar.h"
#include "eigen/Dense.h"


class LineFollower
{
public:
    explicit LineFollower(PinName pin_1,
                          PinName pin_2,
                          float bar_dist,
                          float d_wheel,
                          float L_wheel,
                          float max_motor_vel //rps 
                          );

    virtual ~LineFollower();

    void setRotationalVelGain(float Kp, float Kp_nl);
    void setMaxWheelVel(float new_max_wheel_vel);
    float getAngRad();
    float getRotationalVelocity();
    float getTranslationalVelocity();
    float getRightWheelVelocity();
    float getLeftWheelVelocity();
    bool isLedActive();

private:

    static constexpr int64_t PERIOD_MUS = 500;

    //geometric parameters
    float m_r_wheel; //wheel radius
    float m_L_wheel; //wheel base

    //Rotational velocity controller
    float m_Kp{3.0f};
    float m_Kp_nl{17.0f};

    //Velocity controller data
    float m_max_motor_vel;
    float m_b;
    float m_max_wheel_vel;
    float m_max_wheel_vel_rads;

    //Wheels velocities
    float m_left_wheel_velocity;
    float m_right_wheel_velocity;

    //Angle line sensor
    float m_sensor_bar_avgAngleRad;
    bool isAnyLedActive{false};

    // Geometric objects
    I2C m_i2c;
    SensorBar m_SensorBar; 
    Eigen::Matrix2f m_Cwheel2robot; // transform robot to wheel
    Eigen::Vector2f m_robot_coord; // contains v and w (robot translational and rotational velocities) RAD/S
    Eigen::Vector2f m_wheel_speed; // w1 w2 (wheel speed) RAD/S

    // Thread objects
    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    // Velocity controller functions
    float ang_cntrl_fcn(const float& Kp, const float& Kp_nl, const float& sensor_bar_avgAngleRad);
    float vel_cntrl_v2_fcn(const float& wheel_speed_max, const float& b, const float& robot_omega, const Eigen::Matrix2f& Cwheel2robot);

    // Thread functions
    void followLine();
    void sendThreadFlag();
};




#endif /* LINE_FOLLOWER_H_ */
