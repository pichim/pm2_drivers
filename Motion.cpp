/*
 * Motion.cpp
 * Copyright (c) 2022, ZHAW
 * All rights reserved.
 */

#include <cmath>
#include <algorithm>
#include "Motion.h"

using namespace std;

const float Motion::DEFAULT_LIMIT = 1.0f;       // default value for limits
const float Motion::MINIMUM_LIMIT = 1.0e-9f;    // smallest value allowed for limits

/**
 * Creates a <code>Motion</code> object.
 * The values for position, velocity and acceleration are set to 0.
 */
Motion::Motion() {
    
    position = 0.0;
    velocity = 0.0f;
    
    profileVelocity = DEFAULT_LIMIT;
    profileAcceleration = DEFAULT_LIMIT;
    profileDeceleration = DEFAULT_LIMIT;
}

/**
 * Creates a <code>Motion</code> object with given values for position and velocity.
 * @param position the initial position value of this motion, given in [m] or [rad].
 * @param velocity the initial velocity value of this motion, given in [m/s] or [rad/s].
 */
Motion::Motion(double position, float velocity) {
    
    this->position = position;
    this->velocity = velocity;
    
    profileVelocity = DEFAULT_LIMIT;
    profileAcceleration = DEFAULT_LIMIT;
    profileDeceleration = DEFAULT_LIMIT;
}

/**
 * Creates a <code>Motion</code> object with given values for position and velocity.
 * @param motion another <code>Motion</code> object to copy the values from.
 */
Motion::Motion(const Motion& motion) {
    
    position = motion.position;
    velocity = motion.velocity;
    
    profileVelocity = motion.profileVelocity;
    profileAcceleration = motion.profileAcceleration;
    profileDeceleration = motion.profileDeceleration;
}

/**
 * Deletes the Motion object.
 */
Motion::~Motion() {}

/**
 * Sets the values for position and velocity.
 * @param position the desired position value of this motion, given in [m] or [rad].
 * @param velocity the desired velocity value of this motion, given in [m/s] or [rad/s].
 */
void Motion::set(double position, float velocity) {
    
    this->position = position;
    this->velocity = velocity;
}

/**
 * Sets the values for position and velocity.
 * @param motion another <code>Motion</code> object to copy the values from.
 */
void Motion::set(const Motion& motion) {
    
    position = motion.position;
    velocity = motion.velocity;
}

/**
 * Sets the position value.
 * @param position the desired position value of this motion, given in [m] or [rad].
 */
void Motion::setPosition(double position) {
    
    this->position = position;
}

/**
 * Gets the position value.
 * @return the position value of this motion, given in [m] or [rad].
 */
double Motion::getPosition() {
    
    return position;
}

/**
 * Sets the velocity value.
 * @param velocity the desired velocity value of this motion, given in [m/s] or [rad/s].
 */
void Motion::setVelocity(float velocity) {
    
    this->velocity = velocity;
}

/**
 * Gets the velocity value.
 * @return the velocity value of this motion, given in [m/s] or [rad/s].
 */
float Motion::getVelocity() {
    
    return velocity;
}

/**
 * Sets the limit for the velocity value.
 * @param profileVelocity the limit of the velocity.
 */
void Motion::setProfileVelocity(float profileVelocity) {
    
    if (profileVelocity > MINIMUM_LIMIT) this->profileVelocity = profileVelocity; else this->profileVelocity = MINIMUM_LIMIT;
}

/**
 * Sets the limit for the acceleration value.
 * @param profileAcceleration the limit of the acceleration.
 */
void Motion::setProfileAcceleration(float profileAcceleration) {
    
    if (profileAcceleration > MINIMUM_LIMIT) this->profileAcceleration = profileAcceleration; else this->profileAcceleration = MINIMUM_LIMIT;
}

/**
 * Sets the limit for the deceleration value.
 * @param profileDeceleration the limit of the deceleration.
 */
void Motion::setProfileDeceleration(float profileDeceleration) {
    
    if (profileDeceleration > MINIMUM_LIMIT) this->profileDeceleration = profileDeceleration; else this->profileDeceleration = MINIMUM_LIMIT;
}

/**
 * Sets the limits for velocity, acceleration and deceleration values.
 * @param profileVelocity the limit of the velocity.
 * @param profileAcceleration the limit of the acceleration.
 * @param profileDeceleration the limit of the deceleration.
 */
void Motion::setLimits(float profileVelocity, float profileAcceleration, float profileDeceleration) {
    
    if (profileVelocity > MINIMUM_LIMIT) this->profileVelocity = profileVelocity; else this->profileVelocity = MINIMUM_LIMIT;
    if (profileAcceleration > MINIMUM_LIMIT) this->profileAcceleration = profileAcceleration; else this->profileAcceleration = MINIMUM_LIMIT;
    if (profileDeceleration > MINIMUM_LIMIT) this->profileDeceleration = profileDeceleration; else this->profileDeceleration = MINIMUM_LIMIT;
}

/**
 * Gets the time needed to move to a given target position.
 * @param targetPosition the desired target position given in [m] or [rad].
 * @return the time to move to the target position, given in [s].
 */
float Motion::getTimeToPosition(double targetPosition) {
    
    // calculate position, when velocity is reduced to zero
    
    double stopPosition = (velocity > 0.0f) ? position+(double)(velocity*velocity/profileDeceleration*0.5f) : position-(double)(velocity*velocity/profileDeceleration*0.5f);
    
    if (targetPosition > stopPosition) { // positive velocity required
        
        if (velocity > profileVelocity) { // slow down to profile velocity first
            
            float t1 = (velocity-profileVelocity)/profileDeceleration;
            float t2 = (float)(targetPosition-stopPosition)/profileVelocity;
            float t3 = profileVelocity/profileDeceleration;
            
            return t1+t2+t3;
            
        } else if (velocity > 0.0f) { // speed up to profile velocity
            
            float t1 = (profileVelocity-velocity)/profileAcceleration;
            float t3 = profileVelocity/profileDeceleration;
            float t2 = ((float)(targetPosition-position)-(velocity+profileVelocity)*0.5f*t1)/profileVelocity-0.5f*t3;
            
            if (t2 < 0.0f) {
                float maxVelocity = sqrt((2.0f*(float)(targetPosition-position)*profileAcceleration+velocity*velocity)*profileDeceleration/(profileAcceleration+profileDeceleration));
                t1 = (maxVelocity-velocity)/profileAcceleration;
                t2 = 0.0f;
                t3 = maxVelocity/profileDeceleration;
            }
            
            return t1+t2+t3;
            
        } else { // slow down to zero first, and then speed up to profile velocity
            
            float t1 = -velocity/profileDeceleration;
            float t2 = profileVelocity/profileAcceleration;
            float t4 = profileVelocity/profileDeceleration;
            float t3 = ((float)(targetPosition-position)-velocity*0.5f*t1)/profileVelocity-0.5f*(t2+t4);
            
            if (t3 < 0.0f) {
                float maxVelocity = sqrt((2.0f*(float)(targetPosition-position)*profileDeceleration+velocity*velocity)*profileAcceleration/(profileAcceleration+profileDeceleration));
                t2 = maxVelocity/profileAcceleration;
                t3 = 0.0f;
                t4 = maxVelocity/profileDeceleration;
            }
            
            return t1+t2+t3+t4;
        }
        
    } else { // negative velocity required
        
        if (velocity < -profileVelocity) { // slow down to (negative) profile velocity first
            
            float t1 = (-profileVelocity-velocity)/profileDeceleration;
            float t2 = (float)(stopPosition-targetPosition)/profileVelocity;
            float t3 = profileVelocity/profileDeceleration;
            
            return t1+t2+t3;
            
        } else if (velocity < 0.0f) { // speed up to (negative) profile velocity
            
            float t1 = (velocity+profileVelocity)/profileAcceleration;
            float t3 = profileVelocity/profileDeceleration;
            float t2 = ((float)(position-targetPosition)+(velocity-profileVelocity)*0.5f*t1)/profileVelocity-0.5f*t3;
            
            if (t2 < 0.0f) {
                float minVelocity = -sqrt((-2.0f*(float)(targetPosition-position)*profileAcceleration+velocity*velocity)*profileDeceleration/(profileAcceleration+profileDeceleration));
                t1 = (velocity-minVelocity)/profileAcceleration;
                t2 = 0.0f;
                t3 = -minVelocity/profileDeceleration;
            }
            
            return t1+t2+t3;
            
        } else { // slow down to zero first, and then speed up to (negative) profile velocity
            
            float t1 = velocity/profileDeceleration;
            float t2 = profileVelocity/profileAcceleration;
            float t4 = profileVelocity/profileDeceleration;
            float t3 = (-(float)(targetPosition-position)+velocity*0.5f*t1)/profileVelocity-0.5f*(t2+t4);
            
            if (t3 < 0.0f) {
                float minVelocity = -sqrt((-2.0f*(float)(targetPosition-position)*profileDeceleration+velocity*velocity)*profileAcceleration/(profileAcceleration+profileDeceleration));
                t2 = -minVelocity/profileAcceleration;
                t3 = 0.0f;
                t4 = -minVelocity/profileDeceleration;
            }
            
            return t1+t2+t3+t4;
        }
    }
}

/**
 * Increments the current motion towards a given target velocity.
 * @param targetVelocity the desired target velocity given in [m/s] or [rad/s].
 * @param period the time period to increment the motion values for, given in [s].
 */
void Motion::incrementToVelocity(float targetVelocity, float period) {
    
    if (targetVelocity < -profileVelocity) targetVelocity = -profileVelocity;
    else if (targetVelocity > profileVelocity) targetVelocity = profileVelocity;
    
    if (targetVelocity > 0.0f) {
        
        if (velocity > targetVelocity) { // slow down to target velocity
            
            float t1 = (velocity-targetVelocity)/profileDeceleration;
            
            if (t1 > period) {
                position += (double)((velocity-profileDeceleration*0.5f*period)*period);
                velocity += -profileDeceleration*period;
            } else {
                position += (double)((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += (double)(velocity*(period-t1));
            }
            
        } else if (velocity > 0.0f) { // speed up to target velocity
            
            float t1 = (targetVelocity-velocity)/profileAcceleration;
            
            if (t1 > period) {
                position += (double)((velocity+profileAcceleration*0.5f*period)*period);
                velocity += profileAcceleration*period;
            } else {
                position += (double)((velocity+profileAcceleration*0.5f*t1)*t1);
                velocity += profileAcceleration*t1;
                position += (double)(velocity*(period-t1));
            }
            
        } else { // slow down to zero first, and then speed up to target velocity
            
            float t1 = -velocity/profileDeceleration;
            float t2 = targetVelocity/profileAcceleration;
            
            if (t1 > period) {
                position += (double)((velocity+profileDeceleration*0.5f*period)*period);
                velocity += profileDeceleration*period;
            } else if (t1+t2 > period) {
                position += (double)((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += (double)((velocity+profileAcceleration*0.5f*(period-t1))*(period-t1));
                velocity += profileAcceleration*(period-t1);
            } else {
                position += (double)((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += (double)((velocity+profileAcceleration*0.5f*t2)*t2);
                velocity += profileAcceleration*t2;
                position += (double)(velocity*(period-t1-t2));
            }
        }
        
    } else {
        
        if (velocity < targetVelocity) { // slow down to (negative) target velocity
            
            float t1 = (targetVelocity-velocity)/profileDeceleration;
            
            if (t1 > period) {
                position += (double)((velocity+profileDeceleration*0.5f*period)*period);
                velocity += profileDeceleration*period;
            } else {
                position += (double)((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += (double)(velocity*(period-t1));
            }
            
        } else if (velocity < 0.0f) { // speed up to (negative) target velocity
            
            float t1 = (velocity-targetVelocity)/profileAcceleration;
            
            if (t1 > period) {
                position += (double)((velocity-profileAcceleration*0.5f*period)*period);
                velocity += -profileAcceleration*period;
            } else {
                position += (double)((velocity-profileAcceleration*0.5f*t1)*t1);
                velocity += -profileAcceleration*t1;
                position += (double)(velocity*(period-t1));
            }
            
        } else { // slow down to zero first, and then speed up to (negative) target velocity
            
            float t1 = velocity/profileDeceleration;
            float t2 = -targetVelocity/profileAcceleration;
            
            if (t1 > period) {
                position += (double)((velocity-profileDeceleration*0.5f*period)*period);
                velocity += -profileDeceleration*period;
            } else if (t1+t2 > period) {
                position += (double)((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += (double)((velocity-profileAcceleration*0.5f*(period-t1))*(period-t1));
                velocity += -profileAcceleration*(period-t1);
            } else {
                position += (double)((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += (double)((velocity-profileAcceleration*0.5f*t2)*t2);
                velocity += -profileAcceleration*t2;
                position += (double)(velocity*(period-t1-t2));
            }
        }
    }
}

/**
 * Increments the current motion towards a given target position.
 * @param targetPosition the desired target position given in [m] or [rad].
 * @param period the time period to increment the motion values for, given in [s].
 */
void Motion::incrementToPosition(double targetPosition, float period) {
    
    // calculate position, when velocity is reduced to zero
    
    double stopPosition = (velocity > 0.0f) ? position+(double)(velocity*velocity/profileDeceleration*0.5f) : position-(double)(velocity*velocity/profileDeceleration*0.5f);
    
    if (targetPosition > stopPosition) { // positive velocity required
        
        if (velocity > profileVelocity) { // slow down to profile velocity first
            
            float t1 = (velocity-profileVelocity)/profileDeceleration;
            float t2 = (float)(targetPosition-stopPosition)/profileVelocity;
            float t3 = profileVelocity/profileDeceleration;
            
            if (t1 > period) {
                position += (double)((velocity-profileDeceleration*0.5f*period)*period);
                velocity += -profileDeceleration*period;
            } else if (t1+t2 > period) {
                position += (double)((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += (double)(velocity*(period-t1));
            } else if (t1+t2+t3 > period) {
                position += (double)((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += (double)(velocity*t2);
                position += (double)((velocity-profileDeceleration*0.5f*(period-t1-t2))*(period-t1-t2));
                velocity += -profileDeceleration*(period-t1-t2);
            } else {
                position += (double)((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += (double)(velocity*t2);
                position += (double)((velocity-profileDeceleration*0.5f*t3)*t3);
                velocity += -profileDeceleration*t3;
            }
            
        } else if (velocity > 0.0f) { // speed up to profile velocity
            
            float t1 = (profileVelocity-velocity)/profileAcceleration;
            float t3 = profileVelocity/profileDeceleration;
            float t2 = ((float)(targetPosition-position)-(velocity+profileVelocity)*0.5f*t1)/profileVelocity-0.5f*t3;
            
            if (t2 < 0.0f) {
                float maxVelocity = sqrt((2.0f*(float)(targetPosition-position)*profileAcceleration+velocity*velocity)*profileDeceleration/(profileAcceleration+profileDeceleration));
                t1 = (maxVelocity-velocity)/profileAcceleration;
                t2 = 0.0f;
                t3 = maxVelocity/profileDeceleration;
            }
            
            if (t1 > period) {
                position += (double)((velocity+profileAcceleration*0.5f*period)*period);
                velocity += profileAcceleration*period;
            } else if (t1+t2 > period) {
                position += (double)((velocity+profileAcceleration*0.5f*t1)*t1);
                velocity += profileAcceleration*t1;
                position += (double)(velocity*(period-t1));
            } else if (t1+t2+t3 > period) {
                position += (double)((velocity+profileAcceleration*0.5f*t1)*t1);
                velocity += profileAcceleration*t1;
                position += (double)(velocity*t2);
                position += (double)((velocity-profileDeceleration*0.5f*(period-t1-t2))*(period-t1-t2));
                velocity += -profileDeceleration*(period-t1-t2);
            } else {
                position += (double)((velocity+profileAcceleration*0.5f*t1)*t1);
                velocity += profileAcceleration*t1;
                position += (double)(velocity*t2);
                position += (double)((velocity-profileDeceleration*0.5f*t3)*t3);
                velocity += -profileDeceleration*t3;
            }
            
        } else { // slow down to zero first, and then speed up to profile velocity
            
            float t1 = -velocity/profileDeceleration;
            float t2 = profileVelocity/profileAcceleration;
            float t4 = profileVelocity/profileDeceleration;
            float t3 = ((float)(targetPosition-position)-velocity*0.5f*t1)/profileVelocity-0.5f*(t2+t4);
            
            if (t3 < 0.0f) {
                float maxVelocity = sqrt((2.0f*(float)(targetPosition-position)*profileDeceleration+velocity*velocity)*profileAcceleration/(profileAcceleration+profileDeceleration));
                t2 = maxVelocity/profileAcceleration;
                t3 = 0.0f;
                t4 = maxVelocity/profileDeceleration;
            }
            
            if (t1 > period) {
                position += (double)((velocity+profileDeceleration*0.5f*period)*period);
                velocity += profileDeceleration*period;
            } else if (t1+t2 > period) {
                position += (double)((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += (double)((velocity+profileAcceleration*0.5f*(period-t1))*(period-t1));
                velocity += profileAcceleration*(period-t1);
            } else if (t1+t2+t3 > period) {
                position += (double)((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += (double)((velocity+profileAcceleration*0.5f*t2)*t2);
                velocity += profileAcceleration*t2;
                position += (double)(velocity*(period-t1-t2));
            } else if (t1+t2+t3+t4 > period) {
                position += (double)((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += (double)((velocity+profileAcceleration*0.5f*t2)*t2);
                velocity += profileAcceleration*t2;
                position += (double)(velocity*t3);
                position += (double)((velocity-profileDeceleration*0.5f*(period-t1-t2-t3))*(period-t1-t2-t3));
                velocity += -profileDeceleration*(period-t1-t2-t3);
            } else {
                position += (double)((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += (double)((velocity+profileAcceleration*0.5f*t2)*t2);
                velocity += profileAcceleration*t2;
                position += (double)(velocity*t3);
                position += (double)((velocity-profileDeceleration*0.5f*t4)*t4);
                velocity += -profileDeceleration*t4;
            }
        }
        
    } else { // negative velocity required
        
        if (velocity < -profileVelocity) { // slow down to (negative) profile velocity first
            
            float t1 = (-profileVelocity-velocity)/profileDeceleration;
            float t2 = (float)(stopPosition-targetPosition)/profileVelocity;
            float t3 = profileVelocity/profileDeceleration;
            
            if (t1 > period) {
                position += (double)((velocity+profileDeceleration*0.5f*period)*period);
                velocity += profileDeceleration*period;
            } else if (t1+t2 > period) {
                position += (double)((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += (double)(velocity*(period-t1));
            } else if (t1+t2+t3 > period) {
                position += (double)((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += (double)(velocity*t2);
                position += (double)((velocity+profileDeceleration*0.5f*(period-t1-t2))*(period-t1-t2));
                velocity += profileDeceleration*(period-t1-t2);
            } else {
                position += (double)((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += (double)(velocity*t2);
                position += (double)((velocity+profileDeceleration*0.5f*t3)*t3);
                velocity += profileDeceleration*t3;
            }
            
        } else if (velocity < 0.0f) { // speed up to (negative) profile velocity
            
            float t1 = (velocity+profileVelocity)/profileAcceleration;
            float t3 = profileVelocity/profileDeceleration;
            float t2 = ((float)(position-targetPosition)+(velocity-profileVelocity)*0.5f*t1)/profileVelocity-0.5f*t3;
            
            if (t2 < 0.0f) {
                float minVelocity = -sqrt((-2.0f*(float)(targetPosition-position)*profileAcceleration+velocity*velocity)*profileDeceleration/(profileAcceleration+profileDeceleration));
                t1 = (velocity-minVelocity)/profileAcceleration;
                t2 = 0.0f;
                t3 = -minVelocity/profileDeceleration;
            }
            
            if (t1 > period) {
                position += (double)((velocity-profileAcceleration*0.5f*period)*period);
                velocity += -profileAcceleration*period;
            } else if (t1+t2 > period) {
                position += (double)((velocity-profileAcceleration*0.5f*t1)*t1);
                velocity += -profileAcceleration*t1;
                position += (double)(velocity*(period-t1));
            } else if (t1+t2+t3 > period) {
                position += (double)((velocity-profileAcceleration*0.5f*t1)*t1);
                velocity += -profileAcceleration*t1;
                position += (double)(velocity*t2);
                position += (double)((velocity+profileDeceleration*0.5f*(period-t1-t2))*(period-t1-t2));
                velocity += profileDeceleration*(period-t1-t2);
            } else {
                position += (double)((velocity-profileAcceleration*0.5f*t1)*t1);
                velocity += -profileAcceleration*t1;
                position += (double)(velocity*t2);
                position += (double)((velocity+profileDeceleration*0.5f*t3)*t3);
                velocity += profileDeceleration*t3;
            }
            
        } else { // slow down to zero first, and then speed up to (negative) profile velocity
            
            float t1 = velocity/profileDeceleration;
            float t2 = profileVelocity/profileAcceleration;
            float t4 = profileVelocity/profileDeceleration;
            float t3 = (-(float)(targetPosition-position)+velocity*0.5f*t1)/profileVelocity-0.5f*(t2+t4);
            
            if (t3 < 0.0f) {
                float minVelocity = -sqrt((-2.0f*(float)(targetPosition-position)*profileDeceleration+velocity*velocity)*profileAcceleration/(profileAcceleration+profileDeceleration));
                t2 = -minVelocity/profileAcceleration;
                t3 = 0.0f;
                t4 = -minVelocity/profileDeceleration;
            }
            
            if (t1 > period) {
                position += (double)((velocity-profileDeceleration*0.5f*period)*period);
                velocity += -profileDeceleration*period;
            } else if (t1+t2 > period) {
                position += (double)((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += (double)((velocity-profileAcceleration*0.5f*(period-t1))*(period-t1));
                velocity += -profileAcceleration*(period-t1);
            } else if (t1+t2+t3 > period) {
                position += (double)((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += (double)((velocity-profileAcceleration*0.5f*t2)*t2);
                velocity += -profileAcceleration*t2;
                position += (double)(velocity*(period-t1-t2));
            } else if (t1+t2+t3+t4 > period) {
                position += (double)((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += (double)((velocity-profileAcceleration*0.5f*t2)*t2);
                velocity += -profileAcceleration*t2;
                position += (double)(velocity*t3);
                position += (double)((velocity+profileDeceleration*0.5f*(period-t1-t2-t3))*(period-t1-t2-t3));
                velocity += profileDeceleration*(period-t1-t2-t3);
            } else {
                position += (double)((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += (double)((velocity-profileAcceleration*0.5f*t2)*t2);
                velocity += -profileAcceleration*t2;
                position += (double)(velocity*t3);
                position += (double)((velocity+profileDeceleration*0.5f*t4)*t4);
                velocity += profileDeceleration*t4;
            }
        }
    }
}

