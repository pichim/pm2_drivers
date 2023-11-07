#ifndef LINEAR_CHARACTERISTICS_H_
#define LINEAR_CHARACTERISTICS_H_

class LinearCharacteristics
{

public:

    LinearCharacteristics(float gain, float offset);
    LinearCharacteristics(float x0, float x1, float y0, float y1);
    LinearCharacteristics(float x0, float x1, float y0, float y1, float yMin, float yMax);

    LinearCharacteristics() {};

    virtual ~LinearCharacteristics();

    float operator()(float x)
    {
        return evaluate(x);
    }

    float evaluate(float x);

    void setup(float gain, float offset);
    void setup(float x0, float x1, float y0, float y1);
    void setup(float x0, float x1, float y0, float y1, float yMin, float yMax);

    void correctExistingOffset(float);

private:

    float gain;
    float offset;
    float yMin;
    float yMax;

};

#endif
