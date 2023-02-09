/*
 * ThreadFlag.cpp
 * Copyright (c) 2020, ZHAW
 * All rights reserved.
 */

#include "ThreadFlag.h"

using namespace std;

unsigned int ThreadFlag::threadFlags = 0;

/**
 * Creates a signal object and assignes a unique flag.
 */
ThreadFlag::ThreadFlag()
{
    mutex.lock();

    unsigned int n = 0;
    while ((((1 << n) & threadFlags) > 0) && (n < 30)) n++;
    threadFlag = (1 << n);

    mutex.unlock();
}

/**
 * Deletes the signal object and releases the assigned flag.
 */
ThreadFlag::~ThreadFlag()
{
    mutex.lock();

    threadFlags &= ~threadFlag;

    mutex.unlock();
}

/**
 * Gets the assigned thread flag.
 */
unsigned int ThreadFlag::read()
{
    return threadFlag;
}

/**
 * The empty operator is a shorthand notation of the <code>read()</code> method.
 */
ThreadFlag::operator unsigned int()
{
    return read();
}

