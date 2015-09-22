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
    currentJointState.name.resize(NUM_OF_MOTORS);
    currentJointState.position.resize(NUM_OF_MOTORS);
    currentJointState.velocity.resize(NUM_OF_MOTORS);
    goalJointState.name.resize(NUM_OF_MOTORS);
    goalJointState.position.resize(NUM_OF_MOTORS);
    goalJointState.velocity.resize(NUM_OF_MOTORS);
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

        QTimer* connectionHealthCheckTimer = new QTimer(this);
        connect( connectionHealthCheckTimer, SIGNAL(timeout()), this, SLOT(runConnectionHealthCheck()) );
        connectionHealthCheckTimer->start(5000);

        getSyncFromAXSrv.request.dxlIDs.resize(NUM_OF_MOTORS);
        for (int dxlID = 1; dxlID <= NUM_OF_MOTORS; ++dxlID)
            getSyncFromAXSrv.request.dxlIDs[dxlID-1] = dxlID;
        getSyncFromAXSrv.request.isWord.resize(1);
        getSyncFromAXSrv.request.isWord[0] = true;
        getSyncFromAXClient = n.serviceClient<usb2ax_controller::GetSyncFromAX>("GetSyncFromAX");
        goalJointState.name.resize(NUM_OF_MOTORS);
        goalJointState.position.resize(NUM_OF_MOTORS);
        goalJointState.velocity.resize(NUM_OF_MOTORS);
        goalJointState.effort.resize(NUM_OF_MOTORS);

        QTimer* secondaryDataFeedbackTimer = new QTimer(this);
        connect( secondaryDataFeedbackTimer, SIGNAL(timeout()), this, SLOT(runSecondaryDataFeedback()) );
        secondaryDataFeedbackTimer->start(5000);

        WorkerThread* workerThread = new WorkerThread();
        connect( workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()) );
        workerThread->start();

    }
    else
        mIsMasterRunning = false;
}


//void RosWorker::run() {
//    ros::Rate loop_rate(1);  // Hz
//    while ( ros::ok() )
//    {
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
//}


void RosWorker::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    currentJointState.name = msg->name;
    currentJointState.position = msg->position;
    currentJointState.velocity = msg->velocity;
    currentJointState.effort = msg->effort;
    emit jointStateUpdated(currentJointState);
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


void RosWorker::runSecondaryDataFeedback()
{
    getSyncFromAXSrv.request.startAddress = AX12_GOAL_POSITION_L;
    if ( getSyncFromAXClient.call(getSyncFromAXSrv) )
    {
        for (int i = 0; i < NUM_OF_MOTORS; ++i)
            goalJointState.position[i] = getSyncFromAXSrv.response.values[i];

        getSyncFromAXSrv.request.startAddress = AX12_MOVING_SPEED_L;
        if ( getSyncFromAXClient.call(getSyncFromAXSrv) )
        {
            for (int i = 0; i < NUM_OF_MOTORS; ++i)
                goalJointState.velocity[i] = getSyncFromAXSrv.response.values[i];

            emit secondaryDataUpdated(currentJointState);
        }
    }
}

