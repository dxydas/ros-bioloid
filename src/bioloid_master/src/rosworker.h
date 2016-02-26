#ifndef ROSWORKER_H
#define ROSWORKER_H

#include <qt5/QtCore/QThread>
#include <qt5/QtWidgets/QWidget>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Empty.h"
#include "usb2ax_controller/ReceiveFromAX.h"
#include "usb2ax_controller/SendToAX.h"
#include "usb2ax_controller/ReceiveSyncFromAX.h"
#include "usb2ax_controller/SendSyncToAX.h"
#include "usb2ax_controller/GetMotorParam.h"
#include "usb2ax_controller/SetMotorParam.h"
#include "usb2ax_controller/GetMotorParams.h"
#include "usb2ax_controller/SetMotorParams.h"

class RosWorker : public QObject
{
    Q_OBJECT

public:
    RosWorker(int argc, char* argv[], const char* nodeName, QWidget* parent = 0);
    ~RosWorker();
    void init();
    bool getIsMasterRunning() const { return mIsMasterRunning; }
    sensor_msgs::JointState getCurrentJointState() const { return currentJointState; }
    ros::ServiceClient receiveFromAXClient;
    ros::ServiceClient sendtoAXClient;
    //
    ros::ServiceClient receiveSyncFromAXClient;
    ros::ServiceClient sendSyncToAXClient;
    //
    ros::ServiceClient getMotorCurrentPositionInRadClient;
    ros::ServiceClient getMotorGoalPositionInRadClient;
    ros::ServiceClient setMotorGoalPositionInRadClient;
    //
    ros::ServiceClient getMotorCurrentSpeedInRadPerSecClient;
    ros::ServiceClient getMotorGoalSpeedInRadPerSecClient;
    ros::ServiceClient setMotorGoalSpeedInRadPerSecClient;
    //
    ros::ServiceClient getMotorCurrentTorqueInDecimalClient;
    ros::ServiceClient getMotorMaxTorqueInDecimalClient;
    ros::ServiceClient setMotorMaxTorqueInDecimalClient;
    //
    ros::ServiceClient getMotorCurrentPositionsInRadClient;
    ros::ServiceClient getMotorGoalPositionsInRadClient;
    ros::ServiceClient setMotorGoalPositionsInRadClient;
    //
    ros::ServiceClient getMotorCurrentSpeedsInRadPerSecClient;
    ros::ServiceClient getMotorGoalSpeedsInRadPerSecClient;
    ros::ServiceClient setMotorGoalSpeedsInRadPerSecClient;
    //
    ros::ServiceClient getMotorCurrentTorquesInDecimalClient;
    ros::ServiceClient getMotorMaxTorquesInDecimalClient;
    ros::ServiceClient setMotorMaxTorquesInDecimalClient;
    //
    ros::ServiceClient homeAllMotorsClient;

signals:
    void connectedToRosMaster();
    void disconnectedFromRosMaster();
    void jointStateUpdated(sensor_msgs::JointState js);
    void secondaryDataUpdated(sensor_msgs::JointState js);

public slots:
    void runConnectionHealthCheck();
    void homeAllMotors();
    void setAllMotorTorquesOff();

private:
    int argc;
    char** argv;
    const char* mNodeName;
    bool mIsMasterRunning;
    ros::AsyncSpinner* spinner;
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void goalJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    ros::Subscriber jointStateSub;
    ros::Subscriber goalJointStateSub;
    sensor_msgs::JointState currentJointState;
    sensor_msgs::JointState goalJointState;
};

#endif // ROSWORKER_H
