#include "simplePID.h"


SimplePID::SimplePID(float pGain, float iGain, float dGain, float iMin, float iMax) :
    iMin(iMin), iMax(iMax), pGain(pGain), iGain(iGain), dGain(dGain)
{
    iState = 0.0;
    dState = 0.0;
}


SimplePID::~SimplePID()
{

}


float SimplePID::update(const float& error, const float& position)
{
    float pTerm, iTerm, dTerm;

    pTerm = pGain * error;

    iState += error;
    if (iState > iMax)
        iState = iMax;
    else if (iState < iMin)
        iState = iMin;

    iTerm = iGain*iState;

    dTerm = dGain*(position - dState);
    dState = position;

    return pTerm + iTerm - dTerm;
}

