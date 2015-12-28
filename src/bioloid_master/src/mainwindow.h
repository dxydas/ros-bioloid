#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <qt5/QtCore/QVector>
#include <qt5/QtWidgets/QMainWindow>
#include <qt5/QtWidgets/QWidget>
#include <qt5/QtWidgets/QDockWidget>
#include <qt5/QtWidgets/QMenu>
#include <qt5/QtWidgets/QAction>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QLineEdit>
//
#include "sensor_msgs/JointState.h"
#include <moveit/move_group_interface/move_group.h>
//
#include "rosworker.h"
#include "customlistwidget.h"
#include "outputlog.h"
#include "doubleslider.h"
#include "motorvalueeditor.h"
#include "motordials.h"
#include "moveithandler.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char* argv[], QWidget* parent = 0);
    ~MainWindow();

signals:

public slots:
    void initRosNode();
    void addPose();
    void removePose();

    void planAndExecuteChain();

    void addToQueue();
    void removeFromQueue();

    void nodeConnectedToRosMaster();
    void nodeDisconnectedFromRosMaster();

    void enableMotionButtons();
    void disableMotionButtons();

    void updateJointStateValues(sensor_msgs::JointState js);
    void updateSecondaryRobotValues(sensor_msgs::JointState js);
    void updateJointStateValuesFromPose(const QModelIndex &modelIndex);

    void about();
    void quit();

private:
    void setUpLayout();
    void customiseLayout();
    void connectSignalsAndSlots();

    RosWorker* rosWorker;
    MotorValueEditor* motorValueEditor;
    MotorDials* motorDials;
    MoveItHandler* moveItHandler;
    OutputLog* outputLog;

    QAction* exitAct;
    QAction* aboutQtAct;
    QAction* aboutAct;

    QPushButton* initRosNodeButton;
    QPushButton* addPoseButton;
    QPushButton* removePoseButton;
    QPushButton* setCurrentAsStartStateButton;
    QPushButton* setCurrentAsGoalStateButton;
    QPushButton* planMotionButton;
    QPushButton* executeMotionButton;
    QPushButton* planAndExecuteChainButton;
    QPushButton* addToQueueButton;
    QPushButton* removeFromQueueButton;

    QPushButton* setAllMotorTorquesOffButton;

    QPushButton* saveAvailablePosesFileButton;
    QPushButton* loadAvailablePosesFileButton;
    QPushButton* saveQueuedPosesFileButton;
    QPushButton* loadQueuedPosesFileButton;

    CustomListWidget* availablePosesCustomListWidget;
    CustomListWidget* queuedPosesCustomListWidget;

    QVector<DoubleSlider*> presentPosSliders;
    QVector<QLineEdit*> presentPosLineEdits;
    QVector<QLineEdit*> goalPosLineEdits;
    QVector<QLineEdit*> presentSpeedLineEdits;
    QVector<QLineEdit*> movingSpeedLineEdits;

    QDockWidget* motorFeedbackDockWidget;
    QDockWidget* motorCommandsDockWidget;
    QDockWidget* poseControlDockWidget;
    QDockWidget* fileIoDockWidget;
    QDockWidget* outputLogDockWidget;
    QDockWidget* motorValueEditorDockWidget;
    QDockWidget* motorDialsDockWidget;
};

#endif // MAINWINDOW_H
