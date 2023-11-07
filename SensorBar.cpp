#include "SensorBar.h"

const float SensorBar::TS = 0.004f;                       // period of 1 ms

const char REG_I_ON[16] = {REG_I_ON_0, REG_I_ON_1, REG_I_ON_2, REG_I_ON_3,
                           REG_I_ON_4, REG_I_ON_5, REG_I_ON_6, REG_I_ON_7,
                           REG_I_ON_8, REG_I_ON_9, REG_I_ON_10, REG_I_ON_11,
                           REG_I_ON_12, REG_I_ON_13, REG_I_ON_14, REG_I_ON_15
                          };

const char REG_T_ON[16] = {REG_T_ON_0, REG_T_ON_1, REG_T_ON_2, REG_T_ON_3,
                           REG_T_ON_4, REG_T_ON_5, REG_T_ON_6, REG_T_ON_7,
                           REG_T_ON_8, REG_T_ON_9, REG_T_ON_10, REG_T_ON_11,
                           REG_T_ON_12, REG_T_ON_13, REG_T_ON_14, REG_T_ON_15
                          };

const char REG_OFF[16] = {REG_OFF_0, REG_OFF_1, REG_OFF_2, REG_OFF_3,
                          REG_OFF_4, REG_OFF_5, REG_OFF_6, REG_OFF_7,
                          REG_OFF_8, REG_OFF_9, REG_OFF_10, REG_OFF_11,
                          REG_OFF_12, REG_OFF_13, REG_OFF_14, REG_OFF_15
                         };

const char REG_T_RISE[16] = {0xFF, 0xFF, 0xFF, 0xFF,
                             REG_T_RISE_4, REG_T_RISE_5, REG_T_RISE_6, REG_T_RISE_7,
                             0xFF, 0xFF, 0xFF, 0xFF,
                             REG_T_RISE_12, REG_T_RISE_13, REG_T_RISE_14, REG_T_RISE_15
                            };

const char REG_T_FALL[16] = {0xFF, 0xFF, 0xFF, 0xFF,
                             REG_T_FALL_4, REG_T_FALL_5, REG_T_FALL_6, REG_T_FALL_7,
                             0xFF, 0xFF, 0xFF, 0xFF,
                             REG_T_FALL_12, REG_T_FALL_13, REG_T_FALL_14, REG_T_FALL_15
                            };

SensorBar::SensorBar(I2C& i2c, float distAxisToSensor) : i2c(i2c), thread(osPriorityAboveNormal, 4096)
{
    // Store the received parameters into member variables
    deviceAddress = 0x3E<<1;
    pinInterrupt = 255;
    pinOscillator = 255;
    pinReset = 255;
    invertBits = 0;
    barStrobe = 0; //Default always on

    this->distAxisToSensor = distAxisToSensor;
    lastBarRawValue = lastBarPositionValue = 0;

    angle = avg_angle = 0;
    nrOfLedsActive = 0;
    avg_filter.init(10);
    is_first_avg = true;

    clearBarStrobe();  // to illuminate all the time
    clearInvertBits(); // to make the bar look for a dark line on a reflective surface

    // set up thread
    if (begin()) {
        thread.start(callback(this, &SensorBar::update));
        ticker.attach(callback(this, &SensorBar::sendThreadFlag), std::chrono::microseconds{static_cast<long int>(1.0e6f * TS)});
    }
}

SensorBar::~SensorBar()
{
    ticker.detach();
}

// --- Functions pulled from the SX1509 driver

/*
void SensorBar::debounceConfig(uint8_t configValue)
{
    // First make sure clock is configured
    uint8_t tempuint8_t = readByte(REG_MISC);
    if ((tempuint8_t & 0x70) == 0) {
        tempuint8_t |= (1 << 4);    // Just default to no divider if not set
        writeByte(REG_MISC, tempuint8_t);
    }
    tempuint8_t = readByte(REG_CLOCK);
    if ((tempuint8_t & 0x60) == 0) {
        tempuint8_t |= (1 << 6);    // default to internal osc.
        writeByte(REG_CLOCK, tempuint8_t);
    }

    configValue &= 0b111; // 3-bit value
    writeByte(REG_DEBOUNCE_CONFIG, configValue);
}

void SensorBar::debounceEnable(uint8_t pin)
{
    unsigned int debounceEnable = readWord(REG_DEBOUNCE_ENABLE_B);
    debounceEnable |= (1 << pin);
    writeWord(REG_DEBOUNCE_ENABLE_B, debounceEnable);
}

unsigned int SensorBar::interruptSource(void)
{
    unsigned int intSource = readWord(REG_INTERRUPT_SOURCE_B);
    writeWord(REG_INTERRUPT_SOURCE_B, 0xFFFF);    // Clear interrupts
    return intSource;
}

void SensorBar::configClock(uint8_t oscSource, uint8_t oscPinFunction, uint8_t oscFreqOut, uint8_t oscDivider)
{
    // RegClock constructed as follows:
    //    6:5 - Oscillator frequency souce
    //        00: off, 01: external input, 10: internal 2MHz, 1: reserved
    //    4 - OSCIO pin function
    //        0: input, 1 ouptut
    //    3:0 - Frequency of oscout pin
    //        0: LOW, 0xF: high, else fOSCOUT = FoSC/(2^(RegClock[3:0]-1))
    oscSource = (oscSource & 0b11) << 5;      // 2-bit value, bits 6:5
    oscPinFunction = (oscPinFunction & 1) << 4;   // 1-bit value bit 4
    oscFreqOut = (oscFreqOut & 0b1111);   // 4-bit value, bits 3:0
    uint8_t regClock = oscSource | oscPinFunction | oscFreqOut;
    writeByte(REG_CLOCK, regClock);

    // Config RegMisc[6:4] with oscDivider
    // 0: off, else ClkX = fOSC / (2^(RegMisc[6:4] -1))
    oscDivider = (oscDivider & 0b111) << 4;   // 3-bit value, bits 6:4
    uint8_t regMisc = readByte(REG_MISC);
    regMisc &= ~(0b111 << 4);
    regMisc |= oscDivider;
    writeByte(REG_MISC, regMisc);
}
*/

//Call .setBarStrobing(); to only illuminate while reading line
void SensorBar::setBarStrobe()
{
    barStrobe = 1; //Do strobe
}

//Call .clearBarStrobing(); to illuminate all the time
void SensorBar::clearBarStrobe()
{
    barStrobe = 0; //Always on
}

// .setInvertBits(); to make the bar functions look for a white line on dark surface
void SensorBar::setInvertBits()
{
    invertBits = 1; //Do strobe
}

// .clearInvertBits(); to make the bar look for a dark line on a reflective surface
void SensorBar::clearInvertBits()
{
    invertBits = 0; //Always on
}

//****************************************************************************//
//
//  Bar functions
//
//****************************************************************************//

uint8_t SensorBar::getRaw()
{
    return lastBarRawValue;
}

int8_t SensorBar::getBinaryPosition()
{
    return -lastBarPositionValue;
}

float SensorBar::getAngleRad()
{
    return angle;
}

float SensorBar::getAvgAngleRad()
{
    return avg_angle;
}

uint8_t SensorBar::getNrOfLedsActive()
{
    return nrOfLedsActive;
}

bool SensorBar::isAnyLedActive()
{
    bool retval = false;
    if(nrOfLedsActive != 0) {
        retval =  true;
    }
    return retval;
}

//****************************************************************************//
//
//  Utilities
//
//****************************************************************************//

//Run this once during initialization to configure the SX1509 as a sensor bar
//Returns 1 for success
bool SensorBar::begin(void)
{
    bool returnVar = false;

    // Reset the SX1509
    reset();

    // Communication test. We'll read from two registers with different
    // default values to verify communication.
    unsigned int testRegisters = 0;
    testRegisters = readWord(REG_INTERRUPT_MASK_A);   // This should return 0xFF00
    // Then read a uint8_t that should be 0x00
    if (testRegisters == 0xFF00) {
        //Success!  Configure the device.
        writeByte(REG_DIR_A, 0xFF);
        writeByte(REG_DIR_B, 0xFC);
        writeByte(REG_DATA_B, 0x01);

        returnVar = true;
    }

    return returnVar;
}

// Do a software reset
void SensorBar::reset()
{
    // No hardware option, try software reset
    writeByte(REG_RESET, 0x12);
    writeByte(REG_RESET, 0x34);
}

// readByte(uint8_t registerAddress)
//  This function reads a single uint8_t located at the registerAddress register.
//  - deviceAddress should already be set by the constructor.
//  - Return value is the uint8_t read from registerAddress
//
//  Currently returns 0 if communication has timed out
//
uint8_t SensorBar::readByte(uint8_t registerAddress)
{
    char readValue;
    char data[2] = {registerAddress, 0};
    i2c.write(deviceAddress, data, 1);
    uint8_t val = i2c.read(deviceAddress, &readValue, 1);

    return readValue;
}

// readWord(uint8_t registerAddress)
//  This function will read a two-uint8_t word beginning at registerAddress
//  - A 16-bit unsigned int will be returned.
//      - The msb of the return value will contain the value read from registerAddress
//      - The lsb of the return value will contain the value read from registerAddress + 1
unsigned int SensorBar::readWord(uint8_t registerAddress)
{
    unsigned int readValue;
    unsigned int msb, lsb;
    //unsigned int timeout = RECEIVE_TIMEOUT_VALUE * 2;
    char data[2] = {registerAddress, 0};
    char r_data[2];
    uint8_t val = i2c.write(deviceAddress, data, 1);
    val = i2c.read(deviceAddress, r_data, 2);
    msb = ((unsigned int)r_data[0] & 0x00FF) << 8;
    lsb = ((unsigned int)r_data[1] & 0x00FF);
    readValue = msb | lsb;

    return readValue;
}

// readBytes(uint8_t firstRegisterAddress, uint8_t * destination, uint8_t length)
//  This function reads a series of uint8_ts incrementing from a given address
//  - firstRegsiterAddress is the first address to be read
//  - destination is an array of uint8_ts where the read values will be stored into
//  - length is the number of uint8_ts to be read
//  - No return value.
void SensorBar::readBytes(uint8_t firstRegisterAddress, char * destination, uint8_t length)
{
    char data[2] = {firstRegisterAddress, 0};
    i2c.write(deviceAddress, data, 1);
    uint8_t val = i2c.read(deviceAddress, destination, length);
}

// writeByte(uint8_t registerAddress, uint8_t writeValue)
//  This function writes a single uint8_t to a single register on the SX509.
//  - writeValue is written to registerAddress
//  - deviceAddres should already be set from the constructor
//  - No return value.
void SensorBar::writeByte(uint8_t registerAddress, uint8_t writeValue)
{
    char data[2] = {registerAddress, writeValue};
    i2c.write(deviceAddress, data, 2);
}

// writeWord(uint8_t registerAddress, ungisnged int writeValue)
//  This function writes a two-uint8_t word to registerAddress and registerAddress + 1
//  - the upper uint8_t of writeValue is written to registerAddress
//      - the lower uint8_t of writeValue is written to registerAddress + 1
//  - No return value.
void SensorBar::writeWord(uint8_t registerAddress, unsigned int writeValue)
{
    uint8_t msb, lsb;
    msb = ((writeValue & 0xFF00) >> 8);
    lsb = (writeValue & 0x00FF);
    char data[3] = {registerAddress, msb, lsb};
    i2c.write(deviceAddress, data, 3);
}

// writeBytes(uint8_t firstRegisterAddress, uint8_t * writeArray, uint8_t length)
//  This function writes an array of uint8_ts, beggining at a specific adddress
//  - firstRegisterAddress is the initial register to be written.
//      - All writes following will be at incremental register addresses.
//  - writeArray should be an array of uint8_t values to be written.
//  - length should be the number of uint8_ts to be written.
//  - no return value.
void SensorBar::writeBytes(uint8_t firstRegisterAddress, uint8_t * writeArray, uint8_t length)
{
    char data[10] = {};
    data[0] = firstRegisterAddress;
    for(int i = 0; i < length; i++) {
        data[1+i] = writeArray[i];
    }
    i2c.write(deviceAddress, data, length+1);
}

void SensorBar::update()
{
    while(true) {
        ThisThread::flags_wait_any(threadFlag);

        //Assign values to each bit, -127 to 127, sum, and divide
        int16_t accumulator = 0;
        uint8_t bitsCounted = 0;
        int16_t i;

        //Get the information from the wire, stores in lastBarRawValue
        if( barStrobe == 1 ) {
            writeByte(REG_DATA_B, 0x02); //Turn on IR
            thread_sleep_for(2); // wait_us(2000);
            writeByte(REG_DATA_B, 0x00); //Turn on feedback
        } else {
            writeByte(REG_DATA_B, 0x00); //make sure both IR and indicators are on
        }
        //Operate the I2C machine
        lastBarRawValue = readByte( REG_DATA_A );  //Peel the data off port A

        if( invertBits == 1 ) { //Invert the bits if needed
            lastBarRawValue ^= 0xFF;
        }

        if( barStrobe == 1 ) {
            writeByte(REG_DATA_B, 0x03); //Turn off IR and feedback when done
        }

        //count bits
        for ( i = 0; i < 8; i++ ) {
            if ( ((lastBarRawValue >> i) & 0x01) == 1 ) {
                bitsCounted++;
            }
        }

        //Find the vector value of each positive bit and sum
        for ( i = 7; i > 3; i-- ) { //iterate negative side bits
            if ( ((lastBarRawValue >> i) & 0x01) == 1 ) {
                accumulator += ((-32 * (i - 3)) + 1);
            }
        }
        for ( i = 0; i < 4; i++ ) { //iterate positive side bits
            if ( ((lastBarRawValue >> i) & 0x01) == 1 ) {
                accumulator += ((32 * (4 - i)) - 1);
            }
        }

        if ( bitsCounted > 0 ) {
            lastBarPositionValue = accumulator / bitsCounted;
        } else {
            lastBarPositionValue = 0;
        }

        //Update member variables
        angle = updateAngleRad();
        nrOfLedsActive = updateNrOfLedsActive();

        if(nrOfLedsActive == 0) {
            if(!is_first_avg) {
                avg_filter.reset();
                is_first_avg = true;
            }
        } else {
            if(is_first_avg) {
                is_first_avg = false;
                avg_filter.reset(angle);
            }
            avg_angle = avg_filter.apply(angle);
        }
    }
}

float SensorBar::updateAngleRad()
{
    int8_t binaryPosition  = getBinaryPosition();
    float position = static_cast<float>(binaryPosition) / 127.0f * 0.0445f; // 0.0445 m is half of sensor length
    return atan2f(position, distAxisToSensor);
}

uint8_t SensorBar::updateNrOfLedsActive()
{
    uint8_t bitsCounted = 0;
    uint8_t i;

    //count bits
    for ( i = 0; i < 8; i++ ) {
        if ( ((lastBarRawValue >> i) & 0x01) == 1 ) {
            bitsCounted++;
        }
    }
    return bitsCounted;
}

void SensorBar::sendThreadFlag()
{
    thread.flags_set(threadFlag);
}

//****************************************************************************//
//
//  Circular buffer
//
//****************************************************************************//
/*
//Construct a CircularBuffer type with arguments
//  uint16_t inputSize: number of elements

//mbed has CircularBuffer
namespace name
{

CircularBuffer::CircularBuffer(uint16_t inputSize)
{
    cBufferData = new int16_t[inputSize];
    cBufferLastPtr = 0;
    cBufferElementsUsed = 0;
    cBufferSize = inputSize;
}

CircularBuffer::~CircularBuffer()
{
    delete[] cBufferData;
}

//Get an element at some depth into the circular buffer
//zero is the push location.  Max is cBufferSize - 1
//
//Arguments:
//  uint16_t elementNum: number of element in
//
int16_t CircularBuffer::getElement( uint16_t elementNum )
{
    //Translate elementNum into terms of cBufferLastPtr.
    int16_t virtualElementNum;
    virtualElementNum = cBufferLastPtr - elementNum;
    if( virtualElementNum < 0 ) {
        virtualElementNum += cBufferSize;
    }

    //Output the value
    return cBufferData[virtualElementNum];
}

//Put a new element into the buffer.
//This also expands the size up to the max size
//Arguments:
//
//  int16_t elementVal: value of new element
//
void CircularBuffer::pushElement( int16_t elementVal )
{
    //inc. the pointer
    cBufferLastPtr++;

    //deal with roll
    if( cBufferLastPtr >= cBufferSize ) {
        cBufferLastPtr = 0;
    }

    //write data
    cBufferData[cBufferLastPtr] = elementVal;

    //increase length up to cBufferSize
    if( cBufferElementsUsed < cBufferSize ) {
        cBufferElementsUsed++;
    }
}

//Averages the last n numbers and provides that.  Discards fractions
int16_t CircularBuffer::averageLast( uint16_t numElements )
{
    //Add up all the elements
    int32_t accumulator = 0;
    int8_t i;
    for( i = 0; i < numElements; i++ ) {
        accumulator += getElement( i );
    }
    //Divide by number of elements
    accumulator /= numElements;
    return accumulator;
}

//Returns the current size of the buffer
uint16_t CircularBuffer::recordLength()
{
    return cBufferElementsUsed;
}

}
*/
