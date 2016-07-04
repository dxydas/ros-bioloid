#ifndef AX_JOINT_CONTROLLER_H
#define AX_JOINT_CONTROLLER_H

#include <map>
#include <vector>
#include <stdexcept>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "std_srvs/Empty.h"
#include "usb2ax_controller/ReceiveFromAX.h"
#include "usb2ax_controller/SendToAX.h"
#include "usb2ax_controller/ReceiveSyncFromAX.h"
#include "usb2ax_controller/SendSyncToAX.h"
#include "usb2ax_controller/GetMotorParam.h"
#include "usb2ax_controller/SetMotorParam.h"
#include "usb2ax_controller/GetMotorParams.h"
#include "usb2ax_controller/SetMotorParams.h"
#include "actionlib/server/simple_action_server.h"
#include "controller_manager/controller_manager.h"
#include "bioloidhw.h"

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

int main(int argc, char **argv);

class Ax12ControlTable
{
public:
    explicit Ax12ControlTable();
    static const std::map<int, bool> addressWordMap;
};

class JointController
{
public:
    JointController();
    virtual ~JointController();
    bool init();
    void read();
    void write();
    bool getPositionControlEnabled() const { return positionControlEnabled; }
    void setPositionControlEnabled(bool value) { positionControlEnabled = value; }
    int getDeviceIndex() const {return deviceIndex;}
    void setDeviceIndex(int value) {deviceIndex = value;}
    int getBaudNum() const {return baudNum;}
    void setBaudNum(int value) {baudNum = value;}
    //
    bool receiveFromAX(usb2ax_controller::ReceiveFromAX::Request &req,
                       usb2ax_controller::ReceiveFromAX::Response &res);
    bool sendToAX(usb2ax_controller::SendToAX::Request &req,
                  usb2ax_controller::SendToAX::Response &res);
    //
    bool receiveSyncFromAX(usb2ax_controller::ReceiveSyncFromAX::Request &req,
                           usb2ax_controller::ReceiveSyncFromAX::Response &res);
    bool sendSyncToAX(usb2ax_controller::SendSyncToAX::Request &req,
                      usb2ax_controller::SendSyncToAX::Response &res);
    //
    bool getMotorCurrentPositionInRad(usb2ax_controller::GetMotorParam::Request &req,
                                      usb2ax_controller::GetMotorParam::Response &res);
    bool getMotorGoalPositionInRad(usb2ax_controller::GetMotorParam::Request &req,
                                   usb2ax_controller::GetMotorParam::Response &res);
    bool setMotorGoalPositionInRad(usb2ax_controller::SetMotorParam::Request &req,
                                   usb2ax_controller::SetMotorParam::Response &res);
    //
    bool getMotorCurrentSpeedInRadPerSec(usb2ax_controller::GetMotorParam::Request &req,
                                         usb2ax_controller::GetMotorParam::Response &res);
    bool getMotorGoalSpeedInRadPerSec(usb2ax_controller::GetMotorParam::Request &req,
                                      usb2ax_controller::GetMotorParam::Response &res);
    bool setMotorGoalSpeedInRadPerSec(usb2ax_controller::SetMotorParam::Request &req,
                                      usb2ax_controller::SetMotorParam::Response &res);
    //
    bool getMotorCurrentTorqueInDecimal(usb2ax_controller::GetMotorParam::Request &req,
                                        usb2ax_controller::GetMotorParam::Response &res);
    bool getMotorTorqueLimitInDecimal(usb2ax_controller::GetMotorParam::Request &req,
                                      usb2ax_controller::GetMotorParam::Response &res);
    bool setMotorTorqueLimitInDecimal(usb2ax_controller::SetMotorParam::Request &req,
                                      usb2ax_controller::SetMotorParam::Response &res);
    //
    bool getMotorCurrentPositionsInRad(usb2ax_controller::GetMotorParams::Request &req,
                                      usb2ax_controller::GetMotorParams::Response &res);
    bool getMotorGoalPositionsInRad(usb2ax_controller::GetMotorParams::Request &req,
                                    usb2ax_controller::GetMotorParams::Response &res);
    bool setMotorGoalPositionsInRad(usb2ax_controller::SetMotorParams::Request &req,
                                    usb2ax_controller::SetMotorParams::Response &res);
    //
    bool getMotorCurrentSpeedsInRadPerSec(usb2ax_controller::GetMotorParams::Request &req,
                                         usb2ax_controller::GetMotorParams::Response &res);
    bool getMotorGoalSpeedsInRadPerSec(usb2ax_controller::GetMotorParams::Request &req,
                                       usb2ax_controller::GetMotorParams::Response &res);
    bool setMotorGoalSpeedsInRadPerSec(usb2ax_controller::SetMotorParams::Request &req,
                                       usb2ax_controller::SetMotorParams::Response &res);
    //
    bool getMotorCurrentTorquesInDecimal(usb2ax_controller::GetMotorParams::Request &req,
                                        usb2ax_controller::GetMotorParams::Response &res);
    bool getMotorTorqueLimitsInDecimal(usb2ax_controller::GetMotorParams::Request &req,
                                       usb2ax_controller::GetMotorParams::Response &res);
    bool setMotorTorqueLimitsInDecimal(usb2ax_controller::SetMotorParams::Request &req,
                                       usb2ax_controller::SetMotorParams::Response &res);
    //
    bool homeAllMotors(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    //
    ros::Publisher jointStatePub;
    ros::Publisher goalJointStatePub;
    BioloidHw* bioloidHw;
    controller_manager::ControllerManager* cm;

private:
    void printCommStatus(int CommStatus);
    void printErrorCode(void);
    float axPositionToRad(int oldValue);
    int radToAxPosition(float oldValue);
    float axSpeedToRadPerSec(int oldValue);
    int radPerSecToAxSpeed(float oldValue);
    float axTorqueToDecimal(int oldValue);
    int decimalToAxTorque(float oldValue);
    bool positionControlEnabled;
    int deviceIndex;
    int baudNum;
    int numOfConnectedMotors;
    std::vector<bool> connectedMotors;
    std::vector<int> directionSign;
    sensor_msgs::JointState joint_state;
    sensor_msgs::JointState goal_joint_state;
    ros::Time timeOfLastGoalJointStatePublication;
    int goalJointStatePublicationPeriodInMSecs;
};

#endif // AX_JOINT_CONTROLLER_H
