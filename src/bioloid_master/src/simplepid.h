#ifndef SIMPLEPID_H
#define SIMPLEPID_H

// PID controller is based on code from article "PID without a PhD" from:
// http://www.embedded.com/design/embedded/4211211/PID-without-a-PhD

class SimplePid
{
public:
    SimplePid();
    SimplePid(float proportionalGain, float integralGain, float derivativeGain,
              float outputMax, float outputMin);
    virtual ~SimplePid();
    float getPGain() const { return pGain; }
    virtual void setPGain(float value) { pGain = value; }
    float getIGain() const { return iGain; }
    virtual void setIGain(float value) { iGain = value; }
    float getDGain() const { return dGain; }
    virtual void setDGain(float value) { dGain = value; }
    float getOutMax() const { return outMax; }
    void setOutMax(float value){ outMax = value; }
    float getOutMin() const { return outMin; }
    void setOutMin(float value) { outMin = value; }
    float update(float error, float position);

private:
    float pGain;
    float iGain;
    float dGain;
    float outMin;
    float outMax;
    float iState;
    float dState;
};

#endif // SIMPLEPID_H
