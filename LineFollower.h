#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_

#include <mbed.h>
#include <math.h>

#include "pm2_drivers/SensorBar.h"
#include "eigen/Dense.h"

class LineFollower
{
public:
    /**
     * @brief Construct a new Line Follower object.
     *
     * @param sda_pin I2C data line pin
     * @param scl_pin I2C clock line pin
     * @param bar_dist Distance between sensor bar and wheelbase.
     * @param d_wheel Diameter of the wheel.
     * @param L_wheel Wheelbase.
     * @param max_motor_vel_rps Maximum motor velocity in rotations per second.
     */
    explicit LineFollower(PinName sda_pin,
                          PinName scl_pin,
                          float bar_dist,
                          float d_wheel,
                          float L_wheel,
                          float max_motor_vel_rps);

    /**
     * @brief Destroy the Line Follower object.
     */
    virtual ~LineFollower();

    /**
     * @brief Set the gains for the rotational velocity controller.
     *
     * @param Kp Proportional gain.
     * @param Kp_nl Non-linear proportional gain.
     */
    void setRotationalVelocityGain(float Kp = 2.0f, float Kp_nl = 17.0f);

    /**
     * @brief Set the maximum wheel velocity.
     *
     * @param max_wheel_vel Maximum wheel velocity.
     */
    void setMaxWheelVelocity(float max_wheel_vel);

    /**
     * @brief Get the angle in radians.
     *
     * @return float Angle in radians.
     */
    float getAngleRadians() const;

    /**
     * @brief Get the angle in degrees.
     *
     * @return float Angle in degrees.
     */
    float getAngleDegrees() const;

    /**
     * @brief Get the rotational velocity of robot.
     *
     * @return float Rotational velocity.
     */
    float getRotationalVelocity() const;

    /**
     * @brief Get the translational velocity of robot.
     *
     * @return float Translational velocity.
     */
    float getTranslationalVelocity() const;

    /**
     * @brief Get the right wheel velocity in rotations per second.
     *
     * @return float Right wheel velocity in rotations per second.
     */
    float getRightWheelVelocity() const;

    /**
     * @brief Get the left wheel velocity in rotations per second.
     *
     * @return float Left wheel velocity in rotations per second.
     */
    float getLeftWheelVelocity() const;

    /**
     * @brief Check if the LED is active.
     *
     * @return true If any LED is active.
     * @return false If no LED is active.
     */
    bool isLedActive() const;


private:
    static constexpr int64_t PERIOD_MUS = 10000;

    // geometric parameters
    float m_r_wheel; // wheel radius
    float m_L_wheel; // wheel base

    // rotational velocity controller
    float m_Kp{3.0f};
    float m_Kp_nl{17.0f};

    // velocity controller data
    float m_max_motor_vel;

    float m_rotation_to_wheel_vel;
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
    float vel_cntrl_fcn(float wheel_speed_max, float b, float robot_omega, Eigen::Matrix2f Cwheel2robot);

    // thread functions
    void followLine();
    void sendThreadFlag();
};

#endif /* LINE_FOLLOWER_H_ */