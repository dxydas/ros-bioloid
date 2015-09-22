#ifndef ROSWORKER_H
#define ROSWORKER_H

#include <qt5/QtCore/QThread>
#include <qt5/QtWidgets/QWidget>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "usb2ax_controller/GetSyncFromAX.h"

//class RosCommonNode
//{
//protected:
//    RosCommonNode(int argc, char* argv[], const char* nodeName) :
//        mIsMasterRunning(false)
//    {
//        ros::master::setRetryTimeout(ros::WallDuration(3.0));
//        ros::init(argc, argv, nodeName);
//        if (ros::master::check())
//            mIsMasterRunning = true;
//    }
//    ~RosCommonNode()
//    {
//        ros::shutdown();
//    }
//    bool mIsMasterRunning;
//};

//// This class inherits from RosCommonNode to ensure that the ros::init()
//// function is called BEFORE the ros::NodeHandle is created.
//class RosWorker : RosCommonNode
//{
//public:
//    RosWorker(int argc, char* argv[], const char* nodeName);
//    ros::ServiceClient getAllMotorPositionsInRadClient;

//private:

//};

class WorkerThread : public QThread
{
    Q_OBJECT

public:
    void run();
};

class RosWorker : public QObject//, public RosCommonNode
{
    Q_OBJECT

public:
    RosWorker(int argc, char* argv[], const char* nodeName, QWidget* parent = 0);
    ~RosWorker();
    void init();
    //ros::ServiceClient getAllMotorPositionsInRadClient;
    usb2ax_controller::GetSyncFromAX getSyncFromAXSrv;
    ros::ServiceClient getSyncFromAXClient;
    bool getIsMasterRunning() const { return mIsMasterRunning; }
    sensor_msgs::JointState getCurrentJointState() const { return currentJointState; }

private:
    int argc;
    char** argv;
    const char* mNodeName;
    bool mIsMasterRunning;
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    //WorkerThread* workerThread;
    //ros::NodeHandle n;
    ros::Subscriber jointStateSub;
    sensor_msgs::JointState currentJointState;
    sensor_msgs::JointState goalJointState;

signals:
    void connectedToRosMaster();
    void disconnectedFromRosMaster();
    void jointStateUpdated(sensor_msgs::JointState js);
    void secondaryDataUpdated(sensor_msgs::JointState js);

public slots:
    void runConnectionHealthCheck();
    void runSecondaryDataFeedback();
};

#endif // ROSWORKER_H
