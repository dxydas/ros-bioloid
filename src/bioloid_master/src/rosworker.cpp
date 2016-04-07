#include "rosworker.h"
#include <qt5/QtCore/QTimer>
#include "../../usb2ax_controller/src/ax12ControlTableMacros.h"
#include "commonvars.h"


RosWorker::RosWorker(int argc, char* argv[], const char* nodeName, QWidget* parent) :
    argc(argc), argv(argv), mNodeName(nodeName), QObject(parent), mIsMasterRunning(false)
{
    currentJointState.name.resize(NUM_OF_MOTORS);
    currentJointState.position.resize(NUM_OF_MOTORS);
    currentJointState.velocity.resize(NUM_OF_MOTORS);
    currentJointState.effort.resize(NUM_OF_MOTORS);
    goalJointState.name.resize(NUM_OF_MOTORS);
    goalJointState.position.resize(NUM_OF_MOTORS);
    goalJointState.velocity.resize(NUM_OF_MOTORS);
    goalJointState.effort.resize(NUM_OF_MOTORS);
}


RosWorker::~RosWorker()
{
    if (mIsMasterRunning)
        ros::shutdown();
}


void RosWorker::init()
{
    ros::master::setRetryTimeout(ros::WallDuration(2.0));
    ros::init(argc, argv, mNodeName);

    if (ros::master::check())
    {
        mIsMasterRunning = true;
        ros::NodeHandle n;
        emit connectedToRosMaster();

        jointStateSub = n.subscribe("ax_joint_states", 1000, &RosWorker::jointStateCallback, this);
        goalJointStateSub = n.subscribe("ax_goal_joint_states", 1000, &RosWorker::goalJointStateCallback, this);

        accelSub = n.subscribe("accel", 1000, &RosWorker::accelCallback, this);
        magnetSub = n.subscribe("magnet", 1000, &RosWorker::magnetCallback, this);
        headingSub = n.subscribe("heading", 1000, &RosWorker::headingCallback, this);
        gyroSub = n.subscribe("gyro", 1000, &RosWorker::gyroCallback, this);
        fsrsSub = n.subscribe("fsrs", 1000, &RosWorker::fsrsCallback, this);

        QTimer* connectionHealthCheckTimer = new QTimer(this);
        connect( connectionHealthCheckTimer, SIGNAL(timeout()), this, SLOT(runConnectionHealthCheck()) );
        connectionHealthCheckTimer->start(5000);

        receiveFromAXClient =
                n.serviceClient<usb2ax_controller::ReceiveFromAX>("ReceiveFromAX");
        sendtoAXClient =
                n.serviceClient<usb2ax_controller::SendToAX>("SendToAX");
        //
        receiveSyncFromAXClient =
                n.serviceClient<usb2ax_controller::ReceiveSyncFromAX>("ReceiveSyncFromAX");
        sendSyncToAXClient =
                n.serviceClient<usb2ax_controller::SendSyncToAX>("SendSyncToAX");
        //
        getMotorCurrentPositionInRadClient =
                n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorCurrentPositionInRad");
        getMotorGoalPositionInRadClient =
                n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorGoalPositionInRad");
        setMotorGoalPositionInRadClient =
                n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorGoalPositionInRad");
        //
        getMotorCurrentSpeedInRadPerSecClient =
                n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorCurrentSpeedInRadPerSec");
        getMotorGoalSpeedInRadPerSecClient =
                n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorGoalSpeedInRadPerSec");
        setMotorGoalSpeedInRadPerSecClient =
                n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorGoalSpeedInRadPerSec");
        //
        getMotorCurrentTorqueInDecimalClient =
                n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorCurrentTorqueInDecimal");
        getMotorMaxTorqueInDecimalClient =
                n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorMaxTorqueInDecimal");
        setMotorMaxTorqueInDecimalClient =
                n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorMaxTorqueInDecimal");
        //
        getMotorCurrentPositionsInRadClient =
                n.serviceClient<usb2ax_controller::GetMotorParams>("GetMotorCurrentPositionsInRad");
        getMotorGoalPositionsInRadClient =
                n.serviceClient<usb2ax_controller::GetMotorParams>("GetMotorGoalPositionsInRad");
        setMotorGoalPositionsInRadClient =
                n.serviceClient<usb2ax_controller::SetMotorParams>("SetMotorGoalPositionsInRad");
        //
        getMotorCurrentSpeedsInRadPerSecClient =
                n.serviceClient<usb2ax_controller::GetMotorParams>("GetMotorCurrentSpeedsInRadPerSec");
        getMotorGoalSpeedsInRadPerSecClient =
                n.serviceClient<usb2ax_controller::GetMotorParams>("GetMotorGoalSpeedsInRadPerSec");
        setMotorGoalSpeedsInRadPerSecClient =
                n.serviceClient<usb2ax_controller::SetMotorParams>("SetMotorGoalSpeedsInRadPerSec");
        //
        getMotorCurrentTorquesInDecimalClient =
                n.serviceClient<usb2ax_controller::GetMotorParams>("GetMotorCurrentTorquesInDecimal");
        getMotorMaxTorquesInDecimalClient =
                n.serviceClient<usb2ax_controller::GetMotorParams>("GetMotorMaxTorquesInDecimal");
        setMotorMaxTorquesInDecimalClient =
                n.serviceClient<usb2ax_controller::SetMotorParams>("SetMotorMaxTorquesInDecimal");
        //
        homeAllMotorsClient =
                n.serviceClient<std_srvs::Empty>("HomeAllMotors");

        spinner = new ros::AsyncSpinner(0);
        spinner->start();
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


void RosWorker::accelCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    accel = *msg;
    emit accelDataUpdated(accel);
}


void RosWorker::magnetCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    magnet = *msg;
    emit magnetDataUpdated(magnet);
}


void RosWorker::headingCallback(const std_msgs::Float32::ConstPtr& msg)
{
    heading = *msg;
    emit headingDataUpdated(heading);
}


void RosWorker::gyroCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    gyro = *msg;
    emit gyroDataUpdated(gyro);
}


void RosWorker::fsrsCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    fsrs = *msg;
    emit fsrsDataUpdated(fsrs);
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


void RosWorker::homeAllMotors()
{
    std_srvs::Empty srv;
    homeAllMotorsClient.call(srv);
}


void RosWorker::setAllMotorTorquesOff()
{
    usb2ax_controller::SendToAX srv;
    srv.request.dxlID = 254;
    srv.request.address = AX12_TORQUE_ENABLE;
    srv.request.value = 0;

    sendtoAXClient.call(srv);
}
