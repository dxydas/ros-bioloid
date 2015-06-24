#ifndef SIMPLEPID_H
#define SIMPLEPID_H

// PID controller uses code from article "PID without a PhD" from:
// http://www.embedded.com/design/embedded/4211211/PID-without-a-PhD

class SimplePID
{
public:
    SimplePID(float pGain, float iGain, float dGain, float iMin, float iMax);
    virtual ~SimplePID();
    float update(const float &error, const float &position);
private:
    float pGain;
    float iGain;
    float dGain;
    float iMin;
    float iMax;
    float iState;
    float dState;
};

#endif // SIMPLEPID_H
