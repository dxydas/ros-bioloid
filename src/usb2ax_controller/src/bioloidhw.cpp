#include "bioloidhw.h"


BioloidHw::BioloidHw(std::vector<std::string> jointNames) :
    N(jointNames.size()), name(jointNames),
    cmd(N, 0.0), pos(N, 0.0), vel(N, 0.0), eff(N, 0.0), stateHandles(N), posHandles(N)
{
    // Connect and register the joint state and position interfaces
    for (int i = 0; i < N; ++i)
    {
        stateHandles[i] = new hardware_interface::JointStateHandle(name[i], &pos[i], &vel[i], &eff[i]);
        jointStateInterface.registerHandle(*stateHandles[i]);
        posHandles[i] = new hardware_interface::JointHandle(jointStateInterface.getHandle(name[i]), &cmd[i]);
        jointPosInterface.registerHandle(*posHandles[i]);
    }
    registerInterface(&jointStateInterface);
    registerInterface(&jointPosInterface);
}


double BioloidHw::getCmd(const int &i) const
{
    if (( i >= 0) && (i < cmd.size()) )
        return cmd[i];
    else
        return 0.0;
}


void BioloidHw::setCmd(const int& i, const double& value)
{
    if (( i >= 0) && (i < cmd.size()) )
        cmd[i] = value;
}


double BioloidHw::getPos(const int &i) const
{
    if (( i >= 0) && (i < pos.size()) )
        return pos[i];
    else
        return 0.0;
}


void BioloidHw::setPos(const int& i, const double& value)
{
    if (( i >= 0) && (i < pos.size()) )
        pos[i] = value;
}


double BioloidHw::getVel(const int &i) const
{
    if (( i >= 0) && (i < vel.size()) )
        return vel[i];
    else
        return 0.0;
}


void BioloidHw::setVel(const int& i, const double& value)
{
    if (( i >= 0) && (i < vel.size()) )
        vel[i] = value;
}


double BioloidHw::getEff(const int &i) const
{
    if (( i >= 0) && (i < eff.size()) )
        return eff[i];
    else
        return 0.0;
}


void BioloidHw::setEff(const int& i, const double& value)
{
    if (( i >= 0) && (i < eff.size()) )
        eff[i] = value;
}
