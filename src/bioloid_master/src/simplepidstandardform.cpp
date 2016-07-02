#include "simplepidstandardform.h"


SimplePidStandardForm::SimplePidStandardForm() :
    SimplePidStandardForm(0.0, 1.0, 0.0, 0.0, 0.0)
{
}


SimplePidStandardForm::SimplePidStandardForm(float proportionalGain, float integralTime, float derivativeTime,
                                             float outputMin, float outputMax) :
SimplePid(), iTime(integralTime), dTime(derivativeTime)
{
    // Integral time cannot be zero! Set to 1.0
    if (integralTime == 0)
    {
        integralTime = 1.0;
        iTime = integralTime;
    }
    SimplePid(proportionalGain, proportionalGain/integralTime, proportionalGain*derivativeTime,
              outputMin, outputMax);
}


SimplePidStandardForm::~SimplePidStandardForm()
{
}


void SimplePidStandardForm::setITime(float value)
{
    // iGain = pGain/iTime
    if (value != 0)
    {
        iTime = value;
        SimplePid::setIGain( getPGain()/value );
    }
}


void SimplePidStandardForm::setDTime(float value)
{
    // dGain = pGain*dTime
    dTime = value;
    SimplePid::setDGain( getPGain()*value );
}


void SimplePidStandardForm::setPGain(float value)
{
    SimplePid::setPGain(value);
    updateParameters();
}


void SimplePidStandardForm::setIGain(float value)
{
    SimplePid::setIGain(value);
    updateParameters();
}


void SimplePidStandardForm::setDGain(float value)
{
    SimplePid::setDGain(value);
    updateParameters();
}


void SimplePidStandardForm::updateParameters()
{
    // iTime = pGain/iGain
    // dTime = dGain/pGain
    // Integral time has no effect if integral gain is zero, leave unchanged
    if (getIGain() != 0)
        iTime = getPGain() / getIGain();
    // Derivative time has no effect if proportional gain is zero, leave unchanged
    if (getPGain() != 0)
        dTime = getDGain() / getPGain();
}
