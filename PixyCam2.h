/*
* Pixy cam
* 11/2023
* Maciej
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
#include "PIDLoop.h"

//Standard Bytes
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

#define CCC_SIG_ALL                  0xff // all bits or'ed together
#define CCC_MAX_BLC                  0xff

#define PIXY_CAM_BRIGHTNESS_SYNC     0x10 //16
#define PIXY_SERVO_SYNC              0x12 //18
#define PIXY_LED_SYNC                0x14 //20
#define PIXY_LAMP_SYNC               0x16 //22

#define PRINT_FOR_DEBUG              true

#define FRAMEWIDTH                   315 
#define FRAMEHEIGHT                  207

//This class is to comunicate with Pixy camera, to receive and send data
//Initialization by creating cam object which is member of buffered serial class and defining RX, TX port and baud rate
class PixyCam2 
{
public:
    PixyCam2(PinName TX, PinName RX, int baud);
    virtual ~PixyCam2();

    //Definition of data structure
    typedef struct {
        uint16_t signature;
        uint16_t x;
        uint16_t y;
        uint16_t width;
        uint16_t height;
        int16_t angle; // angle is only available for color coded blocks
        uint8_t index;
        uint8_t age;
    } Block;

    //void state_machine();
    void setPanCntrl(float kp = p_KP, float ki = p_KI, float kd = p_KD);
    void setTiltCntrl(float kp = t_KP, float ki = t_KI, float kd = t_KD);
    uint16_t getSignature();
    uint16_t getX();
    uint16_t getY();
    uint16_t getWidth();
    uint16_t getHeight();
    int16_t getAngle();
    uint8_t getIndex();
    uint8_t getAge();
    float getTiltCommand();
    float getPanCommand();
    void enable(bool enable_var);

    void getBlocks();//uint8_t sigmap=CCC_SIG_ALL, uint8_t maxBlocks=CCC_MAX_BLC);
    
    void setServos(uint16_t s0, uint16_t s1);
    void setCameraBrightness(uint8_t brightness);
    void setLED(uint8_t r, uint8_t g, uint8_t b);
    void setLamp(bool up, bool down);

    //bool is_new_msg;
    
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

    //Pan follower cntrl
    static constexpr float p_KP = 0.75f;
    static constexpr float p_KI = 0.23f / TS;
    static constexpr float p_KD = 0.0f;

    //Tilt follower cntrl
    static constexpr float t_KP = 0.98f;
    static constexpr float t_KI = 0.34f / TS;
    static constexpr float t_KD = 0.0f;

    //Pan/Tilt errors
    int16_t panOffset;
    int16_t tiltOffset;

    //Pan/Tilt update
    uint16_t panUpdate{500};
    uint16_t tiltUpdate{500};

    bool isEnabled{false};

    //Function and clesses
    uint16_t checksum_check(int i);
    uint16_t get_word(int i);
    void msg_read(int i);
    void follow(uint16_t x, uint16_t y);

    PID_Cntrl camPanPID;
    PID_Cntrl camTiltPID;

    //PIDLoop panLoop;
    //PIDLoop tiltLoop;
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