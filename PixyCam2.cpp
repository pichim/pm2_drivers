#include "PixyCam2.h"
#include <cstdint>
#include <cstdio>

// Class constructor
PixyCam2::PixyCam2(PinName Tx,
                   PinName Rx,
                   int baud) : camBufferedSerial(Tx, Rx, baud),
                               camThread()
{
    setServos(panUpdate, tiltUpdate);
    // pm2drivers
    // PM DRIVERS CONTROLLER FUNCTIONS (setPanCntrl, setTiltCntrl, comment out to use my controller)
    setPanCntrl(PixyCam2::p_KP, PixyCam2::p_KI, PixyCam2::p_KD);
    setTiltCntrl(PixyCam2::t_KP, PixyCam2::t_KI, PixyCam2::t_KD);
    iirPanFilter.setup(0.1f, PIXY_FREQ, 1.0f);
    iirTiltFilter.setup(0.1f, PIXY_FREQ, 1.0f);
    camThread.start(callback(this, &PixyCam2::getBlocks));
    camTicker.attach(callback(this, &PixyCam2::sendThreadFlag), std::chrono::microseconds{(int64_t)(PERIOD_MUS)});
};

// Class deconstructor 
PixyCam2::~PixyCam2() 
{
    camTicker.detach();
}

// pm2drivers
// PM DRIVERS CONTROLLER FUNCTIONS (setPanCntrl, setTiltCntrl, comment out to use my controller)
// Pan controller setup
void PixyCam2::setPanCntrl(float kp, float ki, float kd)
{
    camPanPID.setup(kp,
                    ki,
                    kd,
                    TS,
                    MIN_SERVO_POS,
                    MAX_SERVO_POS);
}

// Tilt controller setup
void PixyCam2::setTiltCntrl(float kp, float ki, float kd)
{
    camTiltPID.setup(kp,
                    ki,
                    kd,
                    TS,
                    MIN_SERVO_POS,
                    MAX_SERVO_POS);
}

// Get signature func
uint16_t PixyCam2::getSignature() 
{
    return block.signature;
}

// Get x position func
uint16_t PixyCam2::getX() 
{
    return block.x;
}

// Get y position func
uint16_t PixyCam2::getY() 
{
    return block.y;
}

// Get width func
uint16_t PixyCam2::getWidth() 
{
    return block.width;
}

// Get height func
uint16_t PixyCam2::getHeight() 
{
    return block.height;
}

// Get angle func
int16_t PixyCam2::getAngle() 
{
    return block.angle;
}

// Get index func
uint8_t PixyCam2::getIndex() 
{
    return block.index;
}

// Get age func
uint8_t PixyCam2::getAge() 
{
    return block.age;
}

// Get pan command that is passed to servo
uint16_t PixyCam2::getPanCommand() 
{
    return panUpdate;
}

// Get tilt command that is passed to servo
uint16_t PixyCam2::getTiltCommand() 
{
    return tiltUpdate;
}

// Enable following algorithm
void PixyCam2::followerEnable(bool enable_var) 
{
    isFollowerEnabled = enable_var;
}

// Enable following algorithm
bool PixyCam2::checkForNewMessage() 
{
    return isNewMessage;
}

// Set brightness functions 
void PixyCam2::setCameraBrightness(uint8_t brightness) 
{
    outBuf[0] = PIXY_SEND_SYNC_1;
    outBuf[1] = PIXY_SEND_SYNC_2;
    outBuf[2] = PIXY_CAM_BRIGHTNESS_SYNC;
    outBuf[3] = 1;
    outBuf[4] = brightness;
    camBufferedSerial.write(outBuf, 5);
}

// Set servo functions
void PixyCam2::setServos(uint16_t s0, uint16_t s1) 
{
    outBuf[0] = PIXY_SEND_SYNC_1;
    outBuf[1] = PIXY_SEND_SYNC_2;
    outBuf[2] = PIXY_SERVO_SYNC;
    outBuf[3] = 4;
    *(uint16_t *)(outBuf + 4) = s0;
    *(uint16_t *)(outBuf + 6) = s1;
    camBufferedSerial.write(outBuf, 8);
}

// Set LED functions
void PixyCam2::setLED(uint8_t r, uint8_t g, uint8_t b) 
{
    outBuf[0] = PIXY_SEND_SYNC_1;
    outBuf[1] = PIXY_SEND_SYNC_2;
    outBuf[2] = PIXY_LED_SYNC;
    outBuf[3] = 3;
    outBuf[4] = r;
    outBuf[5] = g;
    outBuf[6] = b;
    camBufferedSerial.write(outBuf, 7);
}

// Set lamp functions
void PixyCam2::setLamp(bool up, bool down) 
{
    outBuf[0] = PIXY_SEND_SYNC_1;
    outBuf[1] = PIXY_SEND_SYNC_2;
    outBuf[2] = PIXY_LAMP_SYNC;
    outBuf[3] = 2;
    outBuf[4] = up;
    outBuf[5] = down;
    camBufferedSerial.write(outBuf, 6);
}

// Main thread function to get the information from camera
void PixyCam2::getBlocks() 
{
    Timer timer;
    timer.start();
    timer.reset();
    int dtime_mus = 0;
    static uint8_t msg_buffer_index;
    static bool msg_start;
    int i;
    int j;

    while(true) {
        ThisThread::flags_wait_any(camThreadFlag);

        outBuf[0] = PIXY_SEND_SYNC_1;
        outBuf[1] = PIXY_SEND_SYNC_2;
        outBuf[2] = CCC_REQUEST_BLOCKS;
        outBuf[3] = CCC_REQUEST_LEN;
        outBuf[4] = CCC_SIG1; // indicate which signatures to receive data from (I want only the signature 1)
        outBuf[5] = 1; // maximum blocks to return (I want only one block to not make the bigger problem, should be the biggest one)

        camBufferedSerial.write(outBuf, OUT_BUFF_SIZE);
        
        dtime_mus = 0;
        msg_start = false;
        // TODO Could be parametrized somehow
        j += 1;
        if (j > 5) {
            isNewMessage = false;
        }
        if (camBufferedSerial.readable()) {
            uint32_t msg_len = camBufferedSerial.read(buffer, BUFF_SIZE);
            for (i = 0; i < msg_len; i++) {
                if (msg_start == true) {
                    msg_buffer[msg_buffer_index++] = buffer[i];
                    if (msg_buffer_index == 16) {
                        checksum = *(uint16_t *)&msg_buffer[0];
                        sum = checksum_check(2);
                        if (sum == checksum) {
                            isNewMessage = true;
                            msg_read(0);
                            TS = std::chrono::duration_cast<std::chrono::microseconds>(timer.elapsed_time()).count() * 1.0e-6f;
                            if(isFollowerEnabled == true) {
                                follow(getX(), getY());
                            }
                            timer.reset();
                            msg_start = false;
                        }
                    }
                }
                //Starting sequence detection
                check_buffer[3] = check_buffer[2];
                check_buffer[2] = check_buffer[1];
                check_buffer[1] = check_buffer[0];
                check_buffer[0] = buffer[i];
                if (check_buffer[3] == PIXY_RECV_SYNC_1 && check_buffer[2] == PIXY_RECV_SYNC_2) {
                    if (check_buffer[1] == CCC_RESPONSE_BLOCKS && check_buffer[0] == CCC_RESPONSE_LEN) { 
                        msg_start = true;
                        checksum = 0;
                        sum = 0;
                        msg_buffer_index = 0;
                        j = 0;
                    }
                    else if (check_buffer[1] == 1 && check_buffer[0] == 4){
                        //i += 6;
                    }
                }
            }
        }
    }
}

// Checksum checking
uint16_t PixyCam2::checksum_check(int i) 
{
    sum = 0;
    for (int j = i; j < (CCC_RESPONSE_LEN + i); j++) {
        sum += msg_buffer[j];
    }  
    return sum;
}

// Reading message function
void PixyCam2::msg_read(int i) 
{
    block.signature = *(uint16_t *)&msg_buffer[i+2];
    block.x = *(uint16_t *)&msg_buffer[i+4];
    block.y = *(uint16_t *)&msg_buffer[i+6];
    block.width = *(uint16_t *)&msg_buffer[i+8];
    block.height = *(uint16_t *)&msg_buffer[i+10];
    block.angle = *(uint16_t *)&msg_buffer[i+12];
    block.index = msg_buffer[i+14];
    block.age = msg_buffer[i+15];
}

// Thread flag sender
void PixyCam2::sendThreadFlag()
{
    camThread.flags_set(camThreadFlag);
}

// pm2drivers
// Object follower
void PixyCam2::follow(uint16_t x_k, uint16_t y_k) 
{
    static const int16_t panThreshold = 16;
    static const int16_t tiltThreshold = 9;
    static float panOffset;
    static float tiltOffset;
    static uint16_t panUpdate_old; 
    static uint16_t tiltUpdate_old; 
    static float x;
    static float y;

    if (TS > TS_MIN) {
    TS = PIXY_FREQ;
    }
    camPanPID.setCoefficients(p_KP, p_KI, p_KD, 0.0f, 0.0f, TS);
    camTiltPID.setCoefficients(t_KP, t_KI, t_KD, 0.0f, 0.0f, TS);

    panOffset = ((float)(FRAMEWIDTH/2.0f) - x_k);
    tiltOffset = (y_k - (float)(FRAMEHEIGHT/2.0f));

    panUpdate = SERVO_SET_POINT + camPanPID.update(panOffset);
    tiltUpdate = SERVO_SET_POINT + camTiltPID.update(tiltOffset);
    
    // Threshold 
    // if (abs(panOffset) > panThreshold) {
    //     panUpdate = SERVO_SET_POINT + camPanPID.update(panOffset); 
    // } else {
    //     panUpdate = SERVO_SET_POINT + panUpdate_old;
    // }
    // if ((abs(tiltOffset) > tiltThreshold)) {
    //     tiltUpdate = SERVO_SET_POINT + camTiltPID.update(tiltOffset);
    // } else {
    //     tiltUpdate = SERVO_SET_POINT + tiltUpdate_old;
    // }

    
    printf("%d, %d \n", panUpdate, tiltUpdate);
    setServos(panUpdate, tiltUpdate);
    panUpdate_old = panUpdate;
    tiltUpdate_old = tiltUpdate;
}


// szar
// MY CONTROLLER WITH IIR FILTER
/*

// Object follower
void PixyCam2::follow(uint16_t x_k, uint16_t y_k) 
{
    static const int16_t panThreshold = 16;
    static const int16_t tiltThreshold = 9;
    static float panOffset;
    static float tiltOffset;
    static uint16_t panUpdate_old; 
    static uint16_t tiltUpdate_old; 
    static float x;
    static float y;

    // IIR filter
    x = iir_filter(x_k);
    y = iir_filter(y_k);

    panOffset = ((float)(FRAMEWIDTH/2.0f) - x);
    tiltOffset = (y - (float)(FRAMEHEIGHT/2.0f));

    if (abs(panOffset) > panThreshold) {
        panUpdate = SERVO_SET_POINT + (int16_t)pi_controller(p_KP, p_KI, TS, panOffset);  
    } else {
        panUpdate = panUpdate_old;
    }
    if ((abs(tiltOffset) > tiltThreshold)) {
        tiltUpdate = SERVO_SET_POINT + (int16_t)pi_controller(t_KP, t_KI, TS, tiltOffset);
    } else {
        tiltUpdate = tiltUpdate_old;
    }   

    setServos(panUpdate, tiltUpdate);
    panUpdate_old = panUpdate;
    tiltUpdate_old = tiltUpdate;
}


// PI controller
float PixyCam2::pi_controller(float kp, float ki, float TS, float offset) 
{
    static float update;
    static float up;
    static float ui;
    up = kp * offset;
    ui = saturate(ui + ki * TS * offset, MIN_SERVO_POS, MAX_SERVO_POS);
    update = saturate(up + ui, MIN_SERVO_POS, MAX_SERVO_POS);
    return update;
}

float PixyCam2::iir_filter(float u_k) 
{
    static float u_k_1;
    static float y_k;
    static float y_k_1;
    static const float tau = 1.0f / (1.0f * M_PI * 4.0f);
    static const float Ts = PIXY_FREQ;

    static float a1 = Ts + 2.0f * tau;
    static float a0 = (Ts - 2.0f *tau) / a1;
    static float b0 = Ts/a1;
    static float b1 = b0;

    y_k = b1 * u_k + b0 * u_k_1 - a0 * y_k_1;

    u_k_1 = u_k;
    y_k_1 = y_k;

    return y_k;
}

float PixyCam2::saturate(float u, float uMin, float uMax)
{
    return (u > uMax) ? uMax : (u < uMin) ? uMin : u;
}

*/