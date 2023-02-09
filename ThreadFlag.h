/*
 * ThreadFlag.h
 * Copyright (c) 2020, ZHAW
 * All rights reserved.
 */

#ifndef THREAD_FLAG_H_
#define THREAD_FLAG_H_

#include <cstdlib>
#include <mbed.h>

/**
 * This class manages the handling of unique thread flags to trigger rtos threads.
 */
class ThreadFlag
{

public:

    ThreadFlag();
    virtual                 ~ThreadFlag();
    virtual unsigned int    read();
    operator unsigned int();

private:

    static unsigned int threadFlags;    // variable that holds all assigned thread flags
    unsigned int        threadFlag;     // thread flag of this object
    Mutex               mutex;          // mutex to lock critical sections
};

#endif /* THREAD_FLAG_H_ */

