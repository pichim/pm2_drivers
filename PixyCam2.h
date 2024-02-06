/*
 * PixyCam2 Class Documentation
 * 
 * This class facilitates communication with the Pixy camera for sending and receiving data.
 * 
 * Created: 11/2023
 * Author: Maciej
*/

#ifndef PIXY_CAM_2_H
#define PIXY_CAM_2_H

#include <mbed.h>
#include <cstdint>
#include <iostream>
#include <cstdio>
#include <stdint.h>
#include "ThreadFlag.h"
#include "PID_Cntrl.h"

// Standard Bytes
#define PIXY_SEND_SYNC_1            0xae
#define PIXY_SEND_SYNC_2            0xc1
#define PIXY_RECV_SYNC_1            0xaf
#define PIXY_RECV_SYNC_2            0xc1
#define CCC_RESPONSE_BLOCKS         0x21
#define CCC_REQUEST_BLOCKS          0x20
#define CCC_REQUEST_LEN             2
#define CCC_RESPONSE_LEN            14

#define BUFF_SIZE                   256
#define OUT_BUFF_SIZE               256

#define CCC_MAX_SIGNATURE           7

// Defines for sigmap:
// You can bitwise "or" these together to make a custom sigmap.
// For example if you're only interested in receiving blocks
// with signatures 1 and 5, you could use a sigmap of 
// PIXY_SIG1 | PIXY_SIG5
#define CCC_SIG1                     1 
#define CCC_SIG2                     2
#define CCC_SIG3                     4
#define CCC_SIG4                     8
#define CCC_SIG5                     16
#define CCC_SIG6                     32
#define CCC_SIG7                     64
#define CCC_COLOR_CODES              128

#define CCC_SIG_ALL                  0xff
#define CCC_MAX_BLC                  0xff

#define PIXY_CAM_BRIGHTNESS_SYNC     0x10 //16
#define PIXY_SERVO_SYNC              0x12 //18
#define PIXY_LED_SYNC                0x14 //20
#define PIXY_LAMP_SYNC               0x16 //22

#define PRINT_FOR_DEBUG              true

#define FRAMEWIDTH                   315 
#define FRAMEHEIGHT                  207

/**
 * @brief Class for communication and control of PixyCam2.
 *
 * This class provides functionalities for communication with PixyCam2,
 * retrieving block information, controlling servos, adjusting camera brightness,
 * setting LED colors, and managing lamp state.
 */
class PixyCam2
{
public:
    /**
     * @brief Construct a new PixyCam2 object.
     *
     * @param TX The pin name for serial transmission.
     * @param RX The pin name for serial reception.
     * @param baud The baud rate for serial communication.
     */
    PixyCam2(PinName TX, PinName RX, int baud);

    /**
     * @brief Destroy the PixyCam2 object.
     */
    virtual ~PixyCam2();

    /**
     * @brief Struct for representing a block of data received from Pixy camera.
     */
    typedef struct {
        uint16_t signature; /**< Signature of the block */
        uint16_t x;         /**< X-coordinate of the block */
        uint16_t y;         /**< Y-coordinate of the block */
        uint16_t width;     /**< Width of the block */
        uint16_t height;    /**< Height of the block */
        int16_t angle;      /**< Angle of the block */
        uint8_t index;      /**< Index of the block */
        uint8_t age;        /**< Age of the block */
    } Block;

    /**
     * @brief Set the PID controller parameters for pan movement.
     *
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    void setPanCntrl(float kp = p_KP, float ki = p_KI, float kd = p_KD);

    /**
     * @brief Set the PID controller parameters for tilt movement.
     *
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    void setTiltCntrl(float kp = t_KP, float ki = t_KI, float kd = t_KD);

    /**
     * @brief Get the signature of the detected block.
     *
     * @return uint16_t The signature value.
     */
    uint16_t getSignature();

    /**
     * @brief Get the X coordinate of the detected block.
     *
     * @return uint16_t The X coordinate value.
     */
    uint16_t getX();

    /**
     * @brief Get the Y coordinate of the detected block.
     *
     * @return uint16_t The Y coordinate value.
     */
    uint16_t getY();

    /**
     * @brief Get the width of the detected block.
     *
     * @return uint16_t The width value.
     */
    uint16_t getWidth();

    /**
     * @brief Get the height of the detected block.
     *
     * @return uint16_t The height value.
     */
    uint16_t getHeight();

    /**
     * @brief Get the angle of the detected block.
     *
     * @return int16_t The angle value.
     */
    int16_t getAngle();

    /**
     * @brief Get the index of the detected block.
     *
     * @return uint8_t The index value.
     */
    uint8_t getIndex();

    /**
     * @brief Get the age of the detected block.
     *
     * @return uint8_t The age value.
     */
    uint8_t getAge();

    /**
     * @brief Get the tilt command.
     *
     * @return uint16_t The tilt command value.
     */
    uint16_t getTiltCommand();

    /**
     * @brief Get the pan command.
     *
     * @return uint16_t The pan command value.
     */
    uint16_t getPanCommand();

    /**
     * @brief Enable/disable the follower mode.
     *
     * @param enable_var Flag to enable/disable follower mode.
     */
    void followerEnable(bool enable_var);

    /**
     * @brief Request and receive blocks information from PixyCam2.
     */
    void getBlocks();

    /**
     * @brief Set the positions of servos.
     *
     * @param s0 Position of servo 0.
     * @param s1 Position of servo 1.
     */
    void setServos(uint16_t s0, uint16_t s1);

    /**
     * @brief Set the camera brightness.
     *
     * @param brightness Brightness level.
     */
    void setCameraBrightness(uint8_t brightness);

    /**
     * @brief Set the LED color.
     *
     * @param r Red value.
     * @param g Green value.
     * @param b Blue value.
     */
    void setLED(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Set the lamp state.
     *
     * @param up Flag to raise the lamp.
     * @param down Flag to lower the lamp.
     */
    void setLamp(bool up, bool down);
 
private:
    static constexpr float PIXY_FREQ = 1.0f/60.0f;
    static constexpr float PERIOD_MUS = 1.0e6f * PIXY_FREQ;
    static constexpr float TS = PIXY_FREQ;
    static constexpr float MIN_SERVO_POS = -500.0f;
    static constexpr float MAX_SERVO_POS = 500.0f;
    static constexpr uint16_t SERVO_SET_POINT = 500;

    //Creation of buffers
    uint8_t *buffer = new uint8_t[BUFF_SIZE];
    uint8_t *check_buffer = new uint8_t[4];
    uint8_t *msg_buffer = new uint8_t[100];
    uint8_t *outBuf = new uint8_t[OUT_BUFF_SIZE];

    //Pixy communication variables
    uint16_t checksum, sum;
    static uint8_t msg_buffer_index;
    static bool msg_start;

    // Pan follower cntrl
    static constexpr float p_KP = 0.75f;
    static constexpr float p_KI = 0.23f / TS;
    static constexpr float p_KD = 0.0f;

    // Tilt follower cntrl
    static constexpr float t_KP = 0.98f;
    static constexpr float t_KI = 0.34f / TS;
    static constexpr float t_KD = 0.0f;

    // Pan/Tilt offsets
    int16_t panOffset;
    int16_t tiltOffset;

    // Pan/Tilt update values
    uint16_t panUpdate{500};
    uint16_t tiltUpdate{500};

    // Enabling follower variable
    bool isFollowerEnabled{false};

    // Function and clesses
    uint16_t checksum_check(int i);
    void msg_read(int i);
    void follow(uint16_t x, uint16_t y);

    PID_Cntrl camPanPID;
    PID_Cntrl camTiltPID;
    BufferedSerial camBufferedSerial;
    Block block;

    //Thread
    ThreadFlag camThreadFlag;
    Thread camThread;
    Ticker camTicker;
    void sendThreadFlag();

    //Debuging tools
#if PRINT_FOR_DEBUG
    Timer camTimer;
#endif

};
#endif /* PIXY_CAM_2_H */