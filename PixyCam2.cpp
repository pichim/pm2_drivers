#include "PixyCam2.h"
#include <cstdio>

//Class constructor
PixyCam2::PixyCam2(PinName Tx,
                   PinName Rx,
                   int baud) : camBufferedSerial(Tx, Rx, baud),
                               camThread()
                               //panLoop(300, 0, 350, true),
                               //tiltLoop(400, 0, 450, true) 
{
    //panLoop.reset();
    //tiltLoop.reset();
    setServos(panUpdate, tiltUpdate);
    setPanCntrl(PixyCam2::p_KP, PixyCam2::p_KI, PixyCam2::p_KD);
    setTiltCntrl(PixyCam2::t_KP, PixyCam2::t_KI, PixyCam2::t_KD);
    camThread.start(callback(this, &PixyCam2::getBlocks));
    camTicker.attach(callback(this, &PixyCam2::sendThreadFlag), std::chrono::microseconds{(int64_t)(PERIOD_MUS)});
};

//Class deconstructor 
PixyCam2::~PixyCam2() {
    camTicker.detach();
}

void PixyCam2::setPanCntrl(float kp, float ki, float kd)
{
    camPanPID.setup(kp,
                    ki,
                    kd,
                    TS,
                    MIN_SERVO_POS,
                    MAX_SERVO_POS);
}

void PixyCam2::setTiltCntrl(float kp, float ki, float kd)
{
    camTiltPID.setup(kp,
                    ki,
                    kd,
                    TS,
                    MIN_SERVO_POS,
                    MAX_SERVO_POS);
}


//Get signature func
uint16_t PixyCam2::getSignature() {
    return block.signature;
}
//Get x position func
uint16_t PixyCam2::getX() {
    return block.x;
}
//Get y position func
uint16_t PixyCam2::getY() {
    return block.y;
}
//Get width func
uint16_t PixyCam2::getWidth() {
    return block.width;
}
//Get height func
uint16_t PixyCam2::getHeight() {
    return block.height;
}
//Get angle func
int16_t PixyCam2::getAngle() {
    return block.angle;
}
//Get index func
uint8_t PixyCam2::getIndex() {
    return block.index;
}
//Get age func
uint8_t PixyCam2::getAge() {
    return block.age;
}

void PixyCam2::enable(bool enable_var)
{
    isEnabled = enable_var;
}
/*
float PixyCam2::getTiltCommand() {
    //return tiltLoop.m_command;
}
float PixyCam2::getPanCommand() {
    //return panLoop.m_command;
}
*/
//Set brightness functions 
void PixyCam2::setCameraBrightness(uint8_t brightness) {
    outBuf[0] = PIXY_SEND_SYNC_1;
    outBuf[1] = PIXY_SEND_SYNC_2;
    outBuf[2] = PIXY_CAM_BRIGHTNESS_SYNC;
    outBuf[3] = 1;
    outBuf[4] = brightness;
    camBufferedSerial.write(outBuf, 5);
}

//Set servo functions
void PixyCam2::setServos(uint16_t s0, uint16_t s1) {
    outBuf[0] = PIXY_SEND_SYNC_1;
    outBuf[1] = PIXY_SEND_SYNC_2;
    outBuf[2] = PIXY_SERVO_SYNC;
    outBuf[3] = 4;
    *(uint16_t *)(outBuf + 4) = s0;
    *(uint16_t *)(outBuf + 6) = s1;
    camBufferedSerial.write(outBuf, 8);
}

//Set LED functions
void PixyCam2::setLED(uint8_t r, uint8_t g, uint8_t b) {
    outBuf[0] = PIXY_SEND_SYNC_1;
    outBuf[1] = PIXY_SEND_SYNC_2;
    outBuf[2] = PIXY_LED_SYNC;
    outBuf[3] = 3;
    outBuf[4] = r;
    outBuf[5] = g;
    outBuf[6] = b;
    camBufferedSerial.write(outBuf, 7);
}

//Set lamp functions
void PixyCam2::setLamp(bool up, bool down) {
    outBuf[0] = PIXY_SEND_SYNC_1;
    outBuf[1] = PIXY_SEND_SYNC_2;
    outBuf[2] = PIXY_LAMP_SYNC;
    outBuf[3] = 2;
    outBuf[4] = up;
    outBuf[5] = down;
    camBufferedSerial.write(outBuf, 6);
}

//Main thread function to get the information from camera
void PixyCam2::getBlocks() {
#if PRINT_FOR_DEBUG    
    Timer timer;
    timer.start();
    timer.reset();
#endif
    static uint8_t msg_buffer_index;
    static bool msg_start;
    uint32_t i;
    int dtime_mus = 0;

    while(true) {
        ThisThread::flags_wait_any(camThreadFlag);

        outBuf[0] = PIXY_SEND_SYNC_1;
        outBuf[1] = PIXY_SEND_SYNC_2;
        outBuf[2] = CCC_REQUEST_BLOCKS;
        outBuf[3] = CCC_REQUEST_LEN;
        outBuf[4] = CCC_SIG1; // indicate which signatures to receive data from (I want only the signature 1)
        outBuf[5] = 1; // maximum blocks to return (I want only one block to not make the bigger problem, should be the biggest one)

        //printf("writ: %d\n", camBufferedSerial.writable());
        camBufferedSerial.write(outBuf, OUT_BUFF_SIZE);
        msg_start = false;
#if PRINT_FOR_DEBUG
        int dtime_mus = 0;
#endif  
        //printf("read: %d\n", camBufferedSerial.readable());
        if (camBufferedSerial.readable()) {
            uint32_t msg_len = camBufferedSerial.read(buffer, BUFF_SIZE);
            //printf("%d \n", msg_len); // 1
            for (i = 0; i < msg_len; i++) {
                //printf("%02x", buffer[i]); //2
                if (msg_start == true) {
                    //printf("msg_start \n");
                    msg_buffer[msg_buffer_index++] = buffer[i];
                    if (msg_buffer_index == 16) {
                        for (int j = 0; j < msg_buffer_index; j++) {
                            //printf("%02x", msg_buffer[j]);
                        }
                        //printf("index = 16 \n");
                        checksum = *(uint16_t *)&msg_buffer[0];
                        //get_word(0);
                        //printf("  %d \n", checksum);
                        sum = checksum_check(2);
                        //printf("%d \n", sum);
                        if (sum == checksum) {
                        //is_new_msg = true;
                            msg_read(0);
                            //printf("new_msg \n"); // 4
#if PRINT_FOR_DEBUG
                            dtime_mus = std::chrono::duration_cast<std::chrono::microseconds>(timer.elapsed_time()).count();
                            timer.reset();
                            //printf("%d \n", dtime_mus);
#endif
                            if(isEnabled == true) {
                                follow(getX(), getY());
                            }
                            //printf("%d, %d, %d \n", block.signature, block.x, block.y);
                            msg_start = false;
                        }
                    }
                }
                //Starting sequence detection
                check_buffer[3] = check_buffer[2];
                check_buffer[2] = check_buffer[1];
                check_buffer[1] = check_buffer[0];
                check_buffer[0] = buffer[i];
                //printf("%02x, %02x, %02x, %02x \n", check_buffer[3], check_buffer[2], check_buffer[1], check_buffer[0]);
                if (check_buffer[3] == PIXY_RECV_SYNC_1 && check_buffer[2] == PIXY_RECV_SYNC_2) {
                    if (check_buffer[1] == CCC_RESPONSE_BLOCKS && check_buffer[0] == CCC_RESPONSE_LEN) { 
                        msg_start = true;
                        checksum = 0;
                        sum = 0;
                        msg_buffer_index = 0;
                    }
                    else if (check_buffer[1] == 1 && check_buffer[0] == 4){
                        //actually I don't have idea what to add here, information is useless and it doesn't interupt the message reading
                    }
                }
            }
#if PRINT_FOR_DEBUG           
        printf("%d, %d, %d, %d \n", dtime_mus, block.signature, block.x, block.y); 
#endif
        }
    }
}

//Checksum checking
uint16_t PixyCam2::checksum_check(int i) {
    sum = 0;
    for (int j = i; j < (CCC_RESPONSE_LEN + i); j++) {
        sum += msg_buffer[j];
    }  
    return sum;
}

void PixyCam2::msg_read(int i) {
    block.signature = *(uint16_t *)&msg_buffer[i+2];
    block.x = *(uint16_t *)&msg_buffer[i+4];
    block.y = *(uint16_t *)&msg_buffer[i+6];
    block.width = *(uint16_t *)&msg_buffer[i+8];
    block.height = *(uint16_t *)&msg_buffer[i+10];
    block.angle = *(uint16_t *)&msg_buffer[i+12];
    block.index = msg_buffer[i+14];
    block.age = msg_buffer[i+15];
    //return 0;
}
/*
//Assigning values to variables
void PixyCam2::msg_read(int i) {
    block.signature = get_word(i+2);
    block.x = get_word(i+4);
    block.y = get_word(i+6);
    block.width = get_word(i+8);
    block.height = get_word(i+10);
    block.angle = get_word(i+12);
    block.index = msg_buffer[i+14];
    block.age = msg_buffer[i+15];
    //return 0;
}
*/
//Function to get uint16_t words of bytes
uint16_t PixyCam2::get_word(int i) {
    uint16_t w;
    uint8_t c;
    w = msg_buffer[i+1];
    c = msg_buffer[i];
    w <<= 8;
    w |= c;
    return w;
}

//Thread flag sender
void PixyCam2::sendThreadFlag()
{
    camThread.flags_set(camThreadFlag);
}

//Object follower
void PixyCam2::follow(uint16_t x, uint16_t y) {
    static int old_x = 0;
    static int old_y = 0;
    //if ((x - old_x) > 4 | (x - old_x) < -4 | (y - old_y) > 4 | (y - old_y) < -4) {
    panOffset = (uint16_t)(FRAMEWIDTH/2) - x;
    tiltOffset = y - (uint16_t)(FRAMEHEIGHT/2);

    panUpdate = SERVO_SET_POINT + (int16_t)camPanPID.update(panOffset);
    tiltUpdate = SERVO_SET_POINT + (int16_t)camTiltPID.update(tiltOffset);
    setServos(panUpdate, tiltUpdate);

    //panLoop.update(panOffset);
    //tiltLoop.update(tiltOffset);
    //printf(" %d \n", tiltOffset);
    //printf("Offset: %d\n", panOffset);
    //printf("%d\n", panUpdate);
    //setServos(panLoop.m_command, tiltLoop.m_command);
    //setServos(uint16_t s0, uint16_t s1)

    //}
    //old_x = x;
    //old_y = y;
}


