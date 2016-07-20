#include "rosworker.h"
#include "commonvars.h"
#include "../../usb2ax_controller/src/ax12ControlTableMacros.h"


RosWorker::RosWorker(int argc, char* argv[], const char* nodeName, OutputLog* outputLog, QWidget* parent) :
    argc(argc), argv(argv), mNodeName(nodeName), outputLog(outputLog), QObject(parent),
    mIsInitialised(false), mIsConnectedToRosMaster(false)
{
    connectionHealthCheckTimer = new QTimer(this);
    connect( connectionHealthCheckTimer, SIGNAL(timeout()), this, SLOT(runConnectionHealthCheck()) );

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
    terminate();
}


void RosWorker::initialise()
{
    if ( !mIsInitialised )
    {
        ros::master::setRetryTimeout(ros::WallDuration(2.0));
        ros::init(argc, argv, mNodeName);

        mIsInitialised = true;
        emit initialised();
        outputLog->appendTimestamped("ROS node initialised");

        connectToRosMaster();
    }
    else
        terminate();
}


void RosWorker::connectToRosMaster()
{
    if ( !mIsConnectedToRosMaster && ros::master::check() )
    {
        ros::NodeHandle n;

        jointStateSub = n.subscribe("ax_joint_states", 1000, &RosWorker::jointStateCallback, this);
        goalJointStateSub = n.subscribe("ax_goal_joint_states", 1000, &RosWorker::goalJointStateCallback, this);

        accelSub = n.subscribe("accel", 1000, &RosWorker::accelCallback, this);
        magnetSub = n.subscribe("magnet", 1000, &RosWorker::magnetCallback, this);
        headingSub = n.subscribe("heading", 1000, &RosWorker::headingCallback, this);
        gyroSub = n.subscribe("gyro", 1000, &RosWorker::gyroCallback, this);
        fsrsSub = n.subscribe("fsrs", 1000, &RosWorker::fsrsCallback, this);

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
        getMotorTorqueLimitInDecimalClient =
                n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorTorqueLimitInDecimal");
        setMotorTorqueLimitInDecimalClient =
                n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorTorqueLimitInDecimal");
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
        getMotorTorqueLimitsInDecimalClient =
                n.serviceClient<usb2ax_controller::GetMotorParams>("GetMotorTorqueLimitsInDecimal");
        setMotorTorqueLimitsInDecimalClient =
                n.serviceClient<usb2ax_controller::SetMotorParams>("SetMotorTorqueLimitsInDecimal");
        //
        homeAllMotorsClient =
                n.serviceClient<std_srvs::Empty>("HomeAllMotors");

        spinner = new ros::AsyncSpinner(0);
        spinner->start();

        listener = new tf::TransformListener;

        mIsConnectedToRosMaster = true;
        emit connectedToRosMaster();
        outputLog->appendTimestamped("ROS node connected to ROS master");

        connectionHealthCheckTimer->start(2000);
    }
}


void RosWorker::terminate()
{
    if (mIsConnectedToRosMaster)
    {
        spinner->stop();
        delete spinner;
        delete listener;
        connectionHealthCheckTimer->stop();

        // Shutdown all Subscribers
        // Once all Subscribers are shut down, ros::ok() becomes false
        jointStateSub.shutdown();
        goalJointStateSub.shutdown();
        accelSub.shutdown();
        magnetSub.shutdown();
        headingSub.shutdown();
        gyroSub.shutdown();
        fsrsSub.shutdown();

        // Shutdown all Service Clients
        receiveFromAXClient.shutdown();
        sendtoAXClient.shutdown();
        receiveSyncFromAXClient.shutdown();
        sendSyncToAXClient.shutdown();
        getMotorCurrentPositionInRadClient.shutdown();
        getMotorGoalPositionInRadClient.shutdown();
        setMotorGoalPositionInRadClient.shutdown();
        getMotorCurrentSpeedInRadPerSecClient.shutdown();
        getMotorGoalSpeedInRadPerSecClient.shutdown();
        setMotorGoalSpeedInRadPerSecClient.shutdown();
        getMotorCurrentTorqueInDecimalClient.shutdown();
        getMotorTorqueLimitInDecimalClient.shutdown();
        setMotorTorqueLimitInDecimalClient.shutdown();
        getMotorCurrentPositionsInRadClient.shutdown();
        getMotorGoalPositionsInRadClient.shutdown();
        setMotorGoalPositionsInRadClient.shutdown();
        getMotorCurrentSpeedsInRadPerSecClient.shutdown();
        getMotorGoalSpeedsInRadPerSecClient.shutdown();
        setMotorGoalSpeedsInRadPerSecClient.shutdown();
        getMotorCurrentTorquesInDecimalClient.shutdown();
        getMotorTorqueLimitsInDecimalClient.shutdown();
        setMotorTorqueLimitsInDecimalClient.shutdown();
        homeAllMotorsClient.shutdown();

        mIsConnectedToRosMaster = false;
        emit disconnectedFromRosMaster();
        outputLog->appendTimestamped("ROS node disconnected from ROS master");
    }

    if (mIsInitialised)
    {
        if (ros::ok())
            ros::shutdown();

        mIsInitialised = false;
        emit terminated();
        outputLog->appendTimestamped("ROS node terminated");
    }
}


void RosWorker::runConnectionHealthCheck()
{
    if ( mIsConnectedToRosMaster && !ros::master::check() )
        terminate();
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
