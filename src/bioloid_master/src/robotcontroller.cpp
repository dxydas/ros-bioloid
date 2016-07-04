#include "robotcontroller.h"
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QInputDialog>
#include <qt5/QtWidgets/QMessageBox>
#include <qt5/QtWidgets/QAbstractButton>
#include "commonvars.h"
#include "../../usb2ax_controller/src/ax12ControlTableMacros.h"


PlanAndExecuteChainWorker::PlanAndExecuteChainWorker(QList<RobotPose> poses, RosWorker* rw) :
    poses(poses), rw(rw)
{
}


PlanAndExecuteChainWorker::~PlanAndExecuteChainWorker()
{
    emit finished();
}


void PlanAndExecuteChainWorker::doWork()
{
    for (int i = 0; i < poses.size(); ++i)
    {
        usb2ax_controller::SetMotorParams srv;
        for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
        {
            srv.request.dxlIDs.push_back(dxlId);
            srv.request.values.push_back( poses[i].jointState.position[dxlId - 1] );
        }
        //log->appendTimestamped("Pose " + QString::number(i));
        rw->setMotorGoalPositionsInRadClient.call(srv);

        // Wait until motion complete
        usb2ax_controller::ReceiveSyncFromAX srv2;
        for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
            srv2.request.dxlIDs.push_back(dxlId);
        srv2.request.startAddress = AX12_MOVING;
        srv2.request.numOfValuesPerMotor = 1;
        bool allMotorsStopped = false;

        while (!allMotorsStopped)
        {
            // Wait 100 msec
            double pauseTimeInSec = 0.1;
//             QEventLoop loop;
//             QTimer::singleShot(pauseTimeInSec*1000, &loop, SLOT(quit()));
//             loop.exec();
            QThread::msleep(pauseTimeInSec*1000);

            // Check for movement
            allMotorsStopped = true;
            rw->receiveSyncFromAXClient.call(srv2);
            for (int i = 0; i < srv2.response.values.size(); ++i)
            {
                if (srv2.response.values[i] == 1)
                {
                    //std::cout << "Still moving!" << std::endl;
                    allMotorsStopped = false;
                }
            }
        }
        //std::cout << "Finished moving!" << std::endl;

        // Wait for specified dwell time
//         QEventLoop loop;
//         QTimer::singleShot(robotPosesList[i].dwellTimeInSec*1000, &loop, SLOT(quit()));
//         loop.exec();
        QThread::msleep(poses[i].dwellTimeInSec*1000);
    }

    emit finished();
}


RobotController::RobotController(RosWorker* rosWorker, QWidget* parent) :
    mRosWorker(rosWorker), QFrame(parent)
{
    QLabel* rosButtonsLabel = new QLabel("ROS control");
    initRosNodeButton = new QPushButton("Initialise ROS node");
    initMoveItHandlerButton = new QPushButton("Initialise MoveIt! handler");
    setCurrentAsStartStateButton = new QPushButton("Set current as start state");
    setCurrentAsGoalStateButton = new QPushButton("Set current as goal state");
    planMotionButton = new QPushButton("Plan motion");
    executeMotionButton = new QPushButton("Execute motion");

    QLabel* poseButtonsLabel = new QLabel("Pose control");
    addPoseButton = new QPushButton("Add pose");
    removePoseButton = new QPushButton("Remove pose");
    addToQueueButton = new QPushButton("Add to queue -->");
    removeFromQueueButton = new QPushButton("<-- Remove from queue");
    planAndExecuteChainButton = new QPushButton("Plan and execute chain");
    testButton = new QPushButton("Test");

    nodeDisconnectedFromRosMaster();

    QGridLayout* rosButtonsSubLayout = new QGridLayout;
    int row = 0;
    int col = 0;
    rosButtonsSubLayout->addWidget(rosButtonsLabel, row++, col);
    rosButtonsSubLayout->addWidget(initRosNodeButton, row++, col);
    rosButtonsSubLayout->addWidget(initMoveItHandlerButton, row++, col);
    rosButtonsSubLayout->addWidget(setCurrentAsStartStateButton, row++, col);
    rosButtonsSubLayout->addWidget(setCurrentAsGoalStateButton, row++, col);
    rosButtonsSubLayout->addWidget(planMotionButton, row++, col);
    rosButtonsSubLayout->addWidget(executeMotionButton, row++, col);
    rosButtonsSubLayout->setAlignment(Qt::AlignTop);

    QGridLayout* poseButtonsSubLayout = new QGridLayout;
    row = 0;
    col = 0;
    poseButtonsSubLayout->addWidget(poseButtonsLabel, row++, col);
    poseButtonsSubLayout->addWidget(addPoseButton, row++, col);
    poseButtonsSubLayout->addWidget(removePoseButton, row++, col);
    poseButtonsSubLayout->addWidget(addToQueueButton, row++, col);
    poseButtonsSubLayout->addWidget(removeFromQueueButton, row++, col);
    poseButtonsSubLayout->addWidget(planAndExecuteChainButton, row++, col);
    poseButtonsSubLayout->addWidget(testButton, row++, col);
    poseButtonsSubLayout->setAlignment(Qt::AlignTop);

    QList<RobotPose> availablePosesList;
    QList<RobotPose> queuedPosesList;
    RobotPose robotPose;
    for (int i = 0; i < 5; ++i)
    {
        std::ostringstream oss;
        oss.width(3);
        oss.fill('0');
        oss.setf(std::ios::fixed, std::ios::floatfield);
        oss << i;
        robotPose.name = "test" + QString::fromStdString(oss.str());
        robotPose.dwellTimeInSec = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 5.0;

        for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
        {
            robotPose.jointState.position[dxlId - 1] =
                    static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (2.555 + 2.56) - 2.56;
            robotPose.jointState.velocity[dxlId - 1] =
                    static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (12.276 + 12.276) - 12.276;
            robotPose.jointState.effort[dxlId - 1] =
                    static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (1.023 + 1.023) - 1.023;
        }

        availablePosesList.push_back(robotPose);
    }

    availablePosesCustomListWidget = new CustomListWidget(availablePosesList, "Available poses", 0, this);
    queuedPosesCustomListWidget = new CustomListWidget(queuedPosesList, "Queued poses", 1, this);

    QWidget* rosButtonsWidget = new QWidget(this);
    rosButtonsWidget->setLayout(rosButtonsSubLayout);

    QWidget* poseButtonsWidget = new QWidget(this);
    poseButtonsWidget->setLayout(poseButtonsSubLayout);

    row = 0;
    col = 0;
    QGridLayout* poseControlLayout = new QGridLayout;
    poseControlLayout->addWidget(rosButtonsWidget, row, col++);
    poseControlLayout->addWidget(availablePosesCustomListWidget, row, col++);
    poseControlLayout->addWidget(poseButtonsWidget, row, col++);
    poseControlLayout->addWidget(queuedPosesCustomListWidget, row++, col++);

    //    col = 0;
    //    layout->addWidget(motorCommandsWidget, row, col++);
    //    layout->addWidget(fileIoWidget, row++, col--, 1, 3);
    //    layout->addWidget(outputLog, row++, col, 1, -1);

    //QWidget* poseControlWidget =  new QWidget(this);
    //poseControlWidget->setLayout(poseControlLayout);
    setLayout(poseControlLayout);

    setObjectName("connectionFrame");
    setStyleSheet("QFrame#connectionFrame { border: 4px solid red; }");
}


void RobotController::addPose()
{
    bool ok;

    // Get name from user
    QString inputName = QInputDialog::getText(this, "New pose",
                                              "Enter new pose name:", QLineEdit::Normal,
                                              "NewPose001", &ok);
    if (ok && !inputName.isEmpty())
    {
        // Get dwell time from user
        double inputTime = QInputDialog::getDouble(this, "Dwell time",
                                                   "Enter new pose dwell time (in secs):", 1.0,
                                                   0.0, 999.999, 3, &ok);
        if (ok)
        {
            RobotPose robotPose;
            robotPose.name = inputName;
            robotPose.jointState = mRosWorker->getCurrentJointState();
            robotPose.dwellTimeInSec = inputTime;
            availablePosesCustomListWidget->add(robotPose);
        }
        else QMessageBox::warning(this, "Pose not added",
                                  "Incorrect pose dwell time.");
    }
    else QMessageBox::warning(this, "Pose not added",
                              "Incorrect pose name.");

}


void RobotController::removePose()
{
    availablePosesCustomListWidget->remove();
}



void RobotController::planAndExecuteChain()
{
    //moveItHandler->planAndExecuteChain(queuedPosesCustomListWidget->getRobotPosesListModel()->getRobotPosesList());


    // Thread test
    QList<RobotPose> robotPosesList = queuedPosesCustomListWidget->getRobotPosesListModel()->getRobotPosesList();

    workerThread = new QThread;
    PlanAndExecuteChainWorker* planAndExecuteChainWorker = new PlanAndExecuteChainWorker(robotPosesList, mRosWorker);
    planAndExecuteChainWorker->moveToThread(workerThread);
    connect( workerThread, SIGNAL(started()), planAndExecuteChainWorker, SLOT(doWork()) );
    connect( planAndExecuteChainWorker, SIGNAL(finished()), workerThread, SLOT(quit()) );
    connect( planAndExecuteChainWorker, SIGNAL(finished()), planAndExecuteChainWorker, SLOT(deleteLater()) );
    connect( workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()) );
    workerThread->start();
}


void RobotController::addToQueue()
{
    queuedPosesCustomListWidget->addFrom(availablePosesCustomListWidget);
}


void RobotController::removeFromQueue()
{
    queuedPosesCustomListWidget->remove();
}


void RobotController::nodeInitialised()
{
    initRosNodeButton->setText("Terminate ROS node");
}


void RobotController::nodeTerminated()
{
    initRosNodeButton->setText("Initialise ROS node");
}


void RobotController::nodeConnectedToRosMaster()
{
    initRosNodeButton->setText("Terminate ROS node");
    enableMotionButtons();
    setStyleSheet("QFrame#connectionFrame { border: 4px solid green; }");
}


void RobotController::nodeDisconnectedFromRosMaster()
{
    initRosNodeButton->setText("Initialise ROS node");
    disableMotionButtons();
    setStyleSheet("QFrame#connectionFrame { border: 4px solid red; }");
}


void RobotController::updateJointStateValuesFromPoseHelper(const QModelIndex &modelIndex)
{
    sensor_msgs::JointState js = availablePosesCustomListWidget->
            getRobotPosesListModel()->getCurrentPose(modelIndex).jointState;
    emit jointStateValuesFromPoseReady(js);
}


void RobotController::enableMotionButtons()
{
    initMoveItHandlerButton->setEnabled(true);
    addPoseButton->setEnabled(true);
    setCurrentAsStartStateButton->setEnabled(true);
    setCurrentAsGoalStateButton->setEnabled(true);
    planMotionButton->setEnabled(true);
    executeMotionButton->setEnabled(true);
    planAndExecuteChainButton->setEnabled(true);
}


void RobotController::disableMotionButtons()
{
    initMoveItHandlerButton->setEnabled(false);
    addPoseButton->setEnabled(false);
    setCurrentAsStartStateButton->setEnabled(false);
    setCurrentAsGoalStateButton->setEnabled(false);
    planMotionButton->setEnabled(false);
    executeMotionButton->setEnabled(false);
    planAndExecuteChainButton->setEnabled(false);
}
