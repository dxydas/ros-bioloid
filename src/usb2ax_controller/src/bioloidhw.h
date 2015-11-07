#ifndef BIOLOIDHW_H
#define BIOLOIDHW_H

#include <string>
#include <vector>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

class BioloidHw : public hardware_interface::RobotHW
{
public:
    BioloidHw(std::vector<std::string> jointNames);
    double getCmd(const int& i) const;
    void setCmd(const int& i, const double& value);
    double getPos(const int& i) const;
    void setPos(const int& i, const double& value);
    double getVel(const int& i) const;
    void setVel(const int& i, const double& value);
    double getEff(const int& i) const;
    void setEff(const int& i, const double& value);

private:
    int N;
    std::vector<std::string> name;
    std::vector<double> cmd;
    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> eff;
    std::vector<hardware_interface::JointStateHandle*> stateHandles;
    std::vector<hardware_interface::JointHandle*> posHandles;
    hardware_interface::JointStateInterface jointStateInterface;
    hardware_interface::PositionJointInterface jointPosInterface;
};

#endif // BIOLOIDHW_H
