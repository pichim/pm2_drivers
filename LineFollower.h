
#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_

#include <mbed.h>
#include <math.h>

#include "pm2_drivers/SensorBar.h"
#include "eigen/Dense.h"

// TODO: Header file needs a proper documentation, similar to DCMotor.h, Servo.h, etc.
//       I did ask ChatGPT for this. Eventhough everything should be calculated in radians internally,
//       think of an interface the students might find easiest. for input and output of this module
//       (i did not really check).

class LineFollower
{
public:
    explicit LineFollower(PinName pin_1,
                          PinName pin_2,
                          float bar_dist,
                          float d_wheel,
                          float L_wheel,
                          float max_motor_vel);

    virtual ~LineFollower();

    // TODO: consider setting default values for the controller gains
    void setRotationalVelocityGain(float Kp, float Kp_nl);
    void setMaxWheelVelocity(float max_wheel_vel);
    float getAngleRadians() const;
    float getAngleDegrees() const;
    float getRotationalVelocity() const;
    float getTranslationalVelocity() const;
    float getRightWheelVelocity() const;
    float getLeftWheelVelocity() const;
    bool isLedActive() const;

private:
    // TODO: 500 mus might be a bit too fast? can't remember what i did in the line follower project, we might
    //       want to discuss this quickly
    static constexpr int64_t PERIOD_MUS = 500;

    // geometric parameters
    float m_r_wheel; // wheel radius
    float m_L_wheel; // wheel base

    // rotational velocity controller
    float m_Kp{3.0f};
    float m_Kp_nl{17.0f};

    // velocity controller data
    float m_max_motor_vel;
    // TODO: What is m_b? this needs a better name
    float m_b;
    float m_max_wheel_vel;
    float m_max_wheel_vel_rads;

    // wheels velocities
    float m_left_wheel_velocity;
    float m_right_wheel_velocity;

    // angle line sensor
    float m_sensor_bar_avgAngleRad;
    bool isAnyLedActive{false};

    I2C m_i2c;
    SensorBar m_SensorBar;

    Eigen::Matrix2f m_Cwheel2robot; // transform robot to wheel
    Eigen::Vector2f m_robot_coord;  // contains w and v (robot rotational and translational velocities)
    Eigen::Vector2f m_wheel_speed;  // w1 w2 (wheel speed) rad/sec

    // thread objects
    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    // velocity controller functions
    float ang_cntrl_fcn(float Kp, float Kp_nl, float sensor_bar_avgAngleRad);
    float vel_cntrl_v2_fcn(float wheel_speed_max, float b, float robot_omega, Eigen::Matrix2f Cwheel2robot);

    // thread functions
    void followLine();
    void sendThreadFlag();
};

#endif /* LINE_FOLLOWER_H_ */
