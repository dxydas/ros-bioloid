#include "rosworker.h"
#include <qt5/QtCore/QTimer>
#include "../usb2ax_controller/src/ax12ControlTableMacros.h"  // TODO: Fix this


#define NUM_OF_MOTORS 18


void WorkerThread::run()
{
    ros::Rate loop_rate(1000);  // Hz
    while ( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}


RosWorker::RosWorker(int argc, char* argv[], const char* nodeName, QWidget* parent) :
    //RosCommonNode(argc, argv, nodeName),
    argc(argc), argv(argv), mNodeName(nodeName), QObject(parent), mIsMasterRunning(false)
{
    currentJointState.name.resize(NUM_OF_MOTORS + 1);
    currentJointState.position.resize(NUM_OF_MOTORS + 1);
    currentJointState.velocity.resize(NUM_OF_MOTORS + 1);
    currentJointState.effort.resize(NUM_OF_MOTORS + 1);
    goalJointState.name.resize(NUM_OF_MOTORS + 1);
    goalJointState.position.resize(NUM_OF_MOTORS + 1);
    goalJointState.velocity.resize(NUM_OF_MOTORS + 1);
    goalJointState.effort.resize(NUM_OF_MOTORS + 1);
}


RosWorker::~RosWorker()
{
    ros::shutdown();

    //workerThread->quit();
    //workerThread->wait();
}


void RosWorker::init()
{
    ros::master::setRetryTimeout(ros::WallDuration(2.0));
    ros::init(argc, argv, mNodeName);
    //ros::start();
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    if (ros::master::check())
    {
        mIsMasterRunning = true;
        ros::NodeHandle n;
        emit connectedToRosMaster();

        jointStateSub = n.subscribe("ax_joint_states", 1000, &RosWorker::jointStateCallback, this);
        goalJointStateSub = n.subscribe("ax_goal_joint_states", 1000, &RosWorker::goalJointStateCallback, this);

        QTimer* connectionHealthCheckTimer = new QTimer(this);
        connect( connectionHealthCheckTimer, SIGNAL(timeout()), this, SLOT(runConnectionHealthCheck()) );
        connectionHealthCheckTimer->start(5000);

        getFromAXClient =
                n.serviceClient<usb2ax_controller::GetFromAX>("GetFromAX");
        sendtoAXClient =
                n.serviceClient<usb2ax_controller::SendToAX>("SendToAX");
        getSyncFromAXClient =
                n.serviceClient<usb2ax_controller::GetSyncFromAX>("GetSyncFromAX");
        sendSyncToAXClient =
                n.serviceClient<usb2ax_controller::SendSyncToAX>("SendSyncToAX");
        getMotorCurrentPositionInRadClient =
                n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorCurrentPositionInRad");
        getMotorGoalPositionInRadClient =
                n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorGoalPositionInRad");
        setMotorGoalPositionInRadClient =
                n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorGoalPositionInRad");
        getMotorCurrentSpeedInRadPerSecClient =
                n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorCurrentSpeedInRadPerSec");
        getMotorGoalSpeedInRadPerSecClient =
                n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorGoalSpeedInRadPerSec");
        setMotorGoalSpeedInRadPerSecClient =
                n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorGoalSpeedInRadPerSec");
        getMotorCurrentTorqueInDecimalClient =
                n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorCurrentTorqueInDecimal");
        setMotorMaxTorqueInDecimalClient =
                n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorMaxTorqueInDecimal");
        getAllMotorGoalPositionsInRadClient =
                n.serviceClient<usb2ax_controller::GetMotorParams>("GetAllMotorGoalPositionsInRad");
        getAllMotorGoalSpeedsInRadPerSecClient =
                n.serviceClient<usb2ax_controller::GetMotorParams>("GetAllMotorGoalSpeedsInRadPerSec");
        getAllMotorMaxTorquesInDecimalClient =
                n.serviceClient<usb2ax_controller::GetMotorParams>("GetAllMotorMaxTorquesInDecimal");
        homeAllMotorsClient =
                n.serviceClient<std_srvs::Empty>("HomeAllMotors");

        WorkerThread* workerThread = new WorkerThread();
        connect( workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()) );
        workerThread->start();
    }
    else
        mIsMasterRunning = false;
}


void RosWorker::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    currentJointState.name = msg->name;
    currentJointState.position = msg->position;
    currentJointState.velocity = msg->velocity;
    currentJointState.effort = msg->effort;
    emit jointStateUpdated(currentJointState);
}


void RosWorker::goalJointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    goalJointState.name = msg->name;
    goalJointState.position = msg->position;
    goalJointState.velocity = msg->velocity;
    goalJointState.effort = msg->effort;
    emit secondaryDataUpdated(goalJointState);
}


void RosWorker::runConnectionHealthCheck()
{
    if (ros::master::check())
    {
        mIsMasterRunning = true;
    }
    else
    {
        emit disconnectedFromRosMaster();
        mIsMasterRunning = false;
    }
}


void RosWorker::setAllMotorTorquesOff()
{
    usb2ax_controller::SendToAX srv;
    srv.request.dxlID = 254;
    srv.request.address = AX12_TORQUE_ENABLE;
    srv.request.value = 0;

    sendtoAXClient.call(srv);
}
