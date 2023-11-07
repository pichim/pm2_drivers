#ifndef IMU_THREAD_H_
#define IMU_THREAD_H_

#include <mbed.h>

#include "Param.h"
#include "ThreadFlag.h"
#include "LSM9DS1/LSM9DS1.h"
#include "LinearCharacteristics3.h"
#include "Mahony.h"

class IMUThread
{
public:
    IMUThread(Data& data, Mutex& mutex);
    virtual ~IMUThread();

    void StartThread();
    
private:
    Data& m_data;
    Mutex& m_dataMutex;
    LSM9DS1 m_imu;
    LinearCharacteristics3 m_magCalib;
    Mahony m_mahony;

    ThreadFlag m_threadFlag;
    Thread m_thread;
    Ticker m_ticker;
    
    void run();
    void sendThreadFlag();
};

#endif /* IMU_THREAD_H_ */
