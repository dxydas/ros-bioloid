#ifndef SIMPLEPIDSTANDARDFORM_H
#define SIMPLEPIDSTANDARDFORM_H

#include "simplepid.h"

class SimplePidStandardForm : public SimplePid
{
public:
    SimplePidStandardForm();
    SimplePidStandardForm(float pGain, float iTime, float dTime,
                          float outMin, float outMax);
    ~SimplePidStandardForm();
    float getITime() const { return iTime; }
    void setITime(float value);
    float getDTime() const{ return dTime; }
    void setDTime(float value);
    void setPGain(float value);
    void setIGain(float value);
    void setDGain(float value);

private:
    float iTime;
    float dTime;
    void updateParameters();
};

#endif // SIMPLEPIDSTANDARDFORM_H
