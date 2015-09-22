#ifndef AX_JOINT_CONTROLLER_H
#define AX_JOINT_CONTROLLER_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Empty.h"
#include "usb2ax_controller/GetFromAX.h"
#include "usb2ax_controller/SendToAX.h"
#include "usb2ax_controller/GetSyncFromAX.h"
#include "usb2ax_controller/SendSyncToAX.h"
#include "usb2ax_controller/GetMotorParam.h"
#include "usb2ax_controller/SetMotorParam.h"
#include <vector>

int main(int argc, char **argv);

class JointController
{
public:
    JointController();
    virtual ~JointController();
    bool init();
    void run();
    int getDeviceIndex() const {return deviceIndex;}
    void setDeviceIndex(int value) {deviceIndex = value;}
    int getBaudNum() const {return baudNum;}
    void setBaudNum(int value) {baudNum = value;}
    ros::Publisher pub;
    bool getFromAX(usb2ax_controller::GetFromAX::Request &req,
                   usb2ax_controller::GetFromAX::Response &res);
    bool sendToAX(usb2ax_controller::SendToAX::Request &req,
                  usb2ax_controller::SendToAX::Response &res);
    bool getSyncFromAX(usb2ax_controller::GetSyncFromAX::Request &req,
                       usb2ax_controller::GetSyncFromAX::Response &res);
    bool sendSyncToAX(usb2ax_controller::SendSyncToAX::Request &req,
                      usb2ax_controller::SendSyncToAX::Response &res);
    bool getMotorCurrentPositionInRad(usb2ax_controller::GetMotorParam::Request &req,
                                      usb2ax_controller::GetMotorParam::Response &res);
    bool setMotorGoalPositionInRad(usb2ax_controller::SetMotorParam::Request &req,
                                   usb2ax_controller::SetMotorParam::Response &res);
    bool getMotorCurrentSpeedInRadPerSec(usb2ax_controller::GetMotorParam::Request &req,
                                         usb2ax_controller::GetMotorParam::Response &res);
    bool setMotorGoalSpeedInRadPerSec(usb2ax_controller::SetMotorParam::Request &req,
                                      usb2ax_controller::SetMotorParam::Response &res);
    bool getMotorCurrentTorqueInDecimal(usb2ax_controller::GetMotorParam::Request &req,
                                        usb2ax_controller::GetMotorParam::Response &res);
    bool setMotorMaxTorqueInDecimal(usb2ax_controller::SetMotorParam::Request &req,
                                    usb2ax_controller::SetMotorParam::Response &res);
    bool getAllMotorPositionsInRad(usb2ax_controller::GetSyncFromAX::Request &req,
                                   usb2ax_controller::GetSyncFromAX::Response &res);
    bool homeAllMotors(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    // Tests
    bool testSending();
    bool testSendingSync();
    bool testValueConversions();
    void testArmWave();

private:
    void printCommStatus(int CommStatus);
    void printErrorCode(void);
    float axPositionToRad(int oldValue);
    int radToAxPosition(float oldValue);
    float axSpeedToRadPerSec(int oldValue);
    int radPerSecToAxSpeed(float oldValue);
    float axTorqueToDecimal(int oldValue);
    int decimalToAxTorque(float oldValue);
    int deviceIndex;
    int baudNum;
    int numOfConnectedMotors;
    std::vector<bool> connectedMotors;
    std::vector<float> positionOffsets;
    std::vector<int> directionSign;
    sensor_msgs::JointState joint_state;
};

#endif // AX_JOINT_CONTROLLER_H
