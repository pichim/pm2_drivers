#include "LinearCharacteristics.h"

LinearCharacteristics::LinearCharacteristics(float gain, float offset)
{
    setup(gain, offset);
}

LinearCharacteristics::LinearCharacteristics(float x0, float x1, float y0, float y1)
{
    setup(x0, x1, y0, y1);
}

LinearCharacteristics::LinearCharacteristics(float x0, float x1, float y0, float y1, float yMin, float yMax)
{
    setup(x0, x1, y0, y1, yMin, yMax);
}

LinearCharacteristics::~LinearCharacteristics() {}

float LinearCharacteristics::evaluate(float x)
{
    float y = this->gain*(x - this->offset);
    if(y > this->yMax)
        y = this->yMax;
    if(y < this->yMin)
        y = this->yMin;
    return y;
}

void LinearCharacteristics::setup(float gain, float offset)
{
    this->gain = gain;
    this->offset = offset;
    this->yMin = -999999.0;
    this->yMax =  999999.0;
}

void LinearCharacteristics::setup(float x0, float x1, float y0, float y1)
{
    this->gain = (y1 - y0)/(x1 - x0);
    this->offset = x1 - y1/this->gain;
    this->yMin = -999999.0;
    this->yMax =  999999.0;
}

void LinearCharacteristics::setup(float x0,float x1, float y0, float y1, float yMin, float yMax)
{
    this->gain = (y1 - y0)/(x1 - x0);
    this->offset = x1 - y1/this->gain;
    this->yMin = yMin;
    this->yMax = yMax;
}

void LinearCharacteristics::correctExistingOffset(float y_offset)
{
    if(gain > 0.0) this->offset = this->offset + 1.0/(this->gain)*y_offset;
}

