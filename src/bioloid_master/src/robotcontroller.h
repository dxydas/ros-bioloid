#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <qt5/QtCore/QThread>
#include <qt5/QtCore/QMutex>
#include <qt5/QtWidgets/QWidget>
#include <qt5/QtWidgets/QPushButton>
#include "sensor_msgs/JointState.h"
#include "rosworker.h"
#include "customlistwidget.h"

class PlanAndExecuteChainWorker : public QObject
{
    Q_OBJECT

public:
    explicit PlanAndExecuteChainWorker(QList<RobotPose> poses, RosWorker* rw, QMutex* mutex);

public slots:
    void doWork();

signals:
    void finished();

private:
    QList<RobotPose> poses;
    RosWorker* rw;
    QMutex* mutex;
};

class RobotController : public QWidget
{
    Q_OBJECT

public:
    explicit RobotController(RosWorker* rosWorker, QWidget* parent = 0);

    QPushButton* initRosNodeButton;
    QPushButton* initMoveItHandlerButton;
    QPushButton* setCurrentAsStartStateButton;
    QPushButton* setCurrentAsGoalStateButton;
    QPushButton* planMotionButton;
    QPushButton* executeMotionButton;

    QPushButton* addPoseButton;
    QPushButton* removePoseButton;
    QPushButton* addToQueueButton;
    QPushButton* removeFromQueueButton;
    QPushButton* planAndExecuteChainButton;
    QPushButton* testButton;

    CustomListWidget* availablePosesCustomListWidget;
    CustomListWidget* queuedPosesCustomListWidget;

signals:
    void jointStateValuesFromPoseReady(sensor_msgs::JointState);

public slots:    
    void addPose();
    void removePose();

    void planAndExecuteChain();

    void addToQueue();
    void removeFromQueue();

    void enableMotionButtons();
    void disableMotionButtons();
    void updateJointStateValuesFromPoseHelper(const QModelIndex &modelIndex);

private:
    void customiseLayout();
    RosWorker* mRosWorker;
    QThread* workerThread;
    QMutex moveMutex;
};

#endif // ROBOTCONTROLLER_H
