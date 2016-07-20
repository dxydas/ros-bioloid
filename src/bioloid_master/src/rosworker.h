#ifndef ROSWORKER_H
#define ROSWORKER_H

#include <qt5/QtCore/QTimer>
#include <qt5/QtWidgets/QWidget>
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_srvs/Empty.h"
#include "usb2ax_controller/ReceiveFromAX.h"
#include "usb2ax_controller/SendToAX.h"
#include "usb2ax_controller/ReceiveSyncFromAX.h"
#include "usb2ax_controller/SendSyncToAX.h"
#include "usb2ax_controller/GetMotorParam.h"
#include "usb2ax_controller/SetMotorParam.h"
#include "usb2ax_controller/GetMotorParams.h"
#include "usb2ax_controller/SetMotorParams.h"
#include "outputlog.h"

class RosWorker : public QObject
{
    Q_OBJECT

public:
    RosWorker(int argc, char* argv[], const char* nodeName, OutputLog* outputLog, QWidget* parent = 0);
    ~RosWorker();
    bool isInitialised() const { return mIsInitialised; }
    bool isConnectedToRosMaster() const { return mIsConnectedToRosMaster; }
    sensor_msgs::JointState getCurrentJointState() const { return currentJointState; }
    sensor_msgs::JointState getGoalJointState() const { return goalJointState; }
//    geometry_msgs::Vector3 getAccel() const { return accel; }
//    geometry_msgs::Vector3 getMagnet() const { return magnet; }
//    std_msgs::Float32 getHeading() const { return heading; }
//    geometry_msgs::Vector3 getGyro() const { return gyro; }
//    std_msgs::Int16MultiArray getFsrs() const { return fsrs; }
    //
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
    ros::ServiceClient getMotorTorqueLimitInDecimalClient;
    ros::ServiceClient setMotorTorqueLimitInDecimalClient;
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
    ros::ServiceClient getMotorTorqueLimitsInDecimalClient;
    ros::ServiceClient setMotorTorqueLimitsInDecimalClient;
    //
    ros::ServiceClient homeAllMotorsClient;
    //
    tf::TransformListener* getListener() const { return listener; }
    void setListener(tf::TransformListener* value) { listener = value; }

signals:
    void initialised();
    bool terminated();
    void connectedToRosMaster();
    void disconnectedFromRosMaster();
    void jointStateUpdated(sensor_msgs::JointState js);
    void secondaryDataUpdated(sensor_msgs::JointState js);
    void accelDataUpdated(geometry_msgs::Vector3 vec);
    void magnetDataUpdated(geometry_msgs::Vector3 vec);
    void headingDataUpdated(std_msgs::Float32 val);
    void gyroDataUpdated(geometry_msgs::Vector3 vec);
    void fsrsDataUpdated(std_msgs::Int16MultiArray arr);

public slots:
    void initialise();
    void connectToRosMaster();
    void terminate();
    void runConnectionHealthCheck();
    void homeAllMotors();
    void setAllMotorTorquesOff();

private:
    int argc;
    char** argv;
    const char* mNodeName;
    OutputLog* outputLog;
    bool mIsInitialised;
    bool mIsConnectedToRosMaster;
    ros::AsyncSpinner* spinner;
    QTimer* connectionHealthCheckTimer;
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void goalJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void accelCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void magnetCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void headingCallback(const std_msgs::Float32::ConstPtr& msg);
    void gyroCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void fsrsCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);
    ros::Subscriber jointStateSub;
    ros::Subscriber goalJointStateSub;
    ros::Subscriber accelSub;
    ros::Subscriber magnetSub;
    ros::Subscriber headingSub;
    ros::Subscriber gyroSub;
    ros::Subscriber fsrsSub;
    sensor_msgs::JointState currentJointState;
    sensor_msgs::JointState goalJointState;
    geometry_msgs::Vector3 accel;
    geometry_msgs::Vector3 magnet;
    std_msgs::Float32 heading;
    geometry_msgs::Vector3 gyro;
    std_msgs::Int16MultiArray fsrs;
    tf::TransformListener* listener;
};

#endif // ROSWORKER_H
