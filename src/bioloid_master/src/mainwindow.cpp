#include "mainwindow.h"
#include <iostream>
#include <sstream>
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QString>
#include <qt5/QtCore/QMetaType>
#include <qt5/QtCore/QTimer>
#include <qt5/QtCore/QMutexLocker>
#include <qt5/QtGui/QIcon>
#include <qt5/QtWidgets/QApplication>
#include <qt5/QtWidgets/QMenuBar>
#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QFrame>
#include <qt5/QtWidgets/QInputDialog>
#include <qt5/QtWidgets/QDoubleSpinBox>
#include <qt5/QtWidgets/QAbstractButton>
#include <qt5/QtWidgets/QAbstractSlider>
#include <qt5/QtWidgets/QSizePolicy>
#include <qt5/QtWidgets/QDesktopWidget>
#include <qt5/QtWidgets/QAbstractScrollArea>
#include <qt5/QtWidgets/QMessageBox>
#include <qt5/QtWidgets/QLayout>
#include "../../usb2ax_controller/src/ax12ControlTableMacros.h"
#include "commonvars.h"

Q_DECLARE_METATYPE(sensor_msgs::JointState)
Q_DECLARE_METATYPE(geometry_msgs::Vector3)
Q_DECLARE_METATYPE(std_msgs::Float32)
Q_DECLARE_METATYPE(std_msgs::Int16MultiArray)


PlanAndExecuteChainWorker::PlanAndExecuteChainWorker(QList<RobotPose> poses, RosWorker* rw, QMutex* mutex) :
    poses(poses), rw(rw), mutex(mutex)
{
}


void PlanAndExecuteChainWorker::doWork()
{
    QMutexLocker locker(mutex);

    //std::cout << "Hello from worker with thread ID: " << thread()->currentThreadId() << std::endl;


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


MainWindow::MainWindow(int argc, char* argv[], QWidget* parent) :
    QMainWindow(parent)
{
    qRegisterMetaType<sensor_msgs::JointState>("JointState");
    qRegisterMetaType<geometry_msgs::Vector3>("Vector3");
    qRegisterMetaType<std_msgs::Float32>("Float32");
    qRegisterMetaType<std_msgs::Int16MultiArray>("Int16MultiArray");

    rosWorker = new RosWorker(argc, argv, "rosoloid_gui", this);
    motorValueEditor = new MotorValueEditor(rosWorker, this);
    motorAddressEditor = new MotorAddressEditor(rosWorker, this);
    motorDials = new MotorDials(rosWorker, this);
    moveItHandler = new MoveItHandler(this);
    outputLog = new OutputLog(this);
    sensorGrapher = new SensorGrapher(rosWorker, this);

    setUpLayout();
    customiseLayout();
    connectSignalsAndSlots();

    outputLog->appendTimestamped("Application started");
}


MainWindow::~MainWindow()
{
}


void MainWindow::setUpLayout()
{
    int row = 0;
    int col = 0;

    QLabel* motorLabel = new QLabel("Motor");
    QLabel* presentPositionAndSelectedPoseLabel = new QLabel("Present position\nand selected pose");
    QLabel* presentPositionLabel = new QLabel("Present\nposition");
    QLabel* goalPositionLabel = new QLabel("Goal\nposition");
    QLabel* presentSpeedLabel = new QLabel("Present\nspeed");
    QLabel* movingSpeedLabel = new QLabel("Moving\nspeed");

    motorLabel->setAlignment(Qt::AlignCenter);
    presentPositionAndSelectedPoseLabel->setAlignment(Qt::AlignCenter);
    presentPositionLabel->setAlignment(Qt::AlignCenter);
    goalPositionLabel->setAlignment(Qt::AlignCenter);
    presentSpeedLabel->setAlignment(Qt::AlignCenter);
    movingSpeedLabel->setAlignment(Qt::AlignCenter);

    presentPositionAndSelectedPoseLabel->setStyleSheet("QLabel { color: royalblue }");
    presentPositionLabel->setStyleSheet("QLabel { color: royalblue }");
    goalPositionLabel->setStyleSheet("QLabel { color: midnightblue }");
    presentSpeedLabel->setStyleSheet("QLabel { color: royalblue }");
    movingSpeedLabel->setStyleSheet("QLabel { color: midnightblue }");

    QGridLayout* motorFeedbackSubLayout = new QGridLayout;
    motorFeedbackSubLayout->addWidget(motorLabel, row, col++);
    ++col;  // Leave a blank column in order to add vline spacer later
    motorFeedbackSubLayout->addWidget(presentPositionAndSelectedPoseLabel, row, col++);
    motorFeedbackSubLayout->addWidget(presentPositionLabel, row, col++);
    ++col;  // For vline
    motorFeedbackSubLayout->addWidget(goalPositionLabel, row, col++);
    ++col;  // For vline
    motorFeedbackSubLayout->addWidget(presentSpeedLabel, row, col++);
    ++col;  // For vline
    motorFeedbackSubLayout->addWidget(movingSpeedLabel, row, col++);

    QVector<QLabel*> motorIdLabels(NUM_OF_MOTORS);
    presentPosSliders.resize(NUM_OF_MOTORS);
    presentPosLineEdits.resize(NUM_OF_MOTORS);
    goalPosLineEdits.resize(NUM_OF_MOTORS);
    presentSpeedLineEdits.resize(NUM_OF_MOTORS);
    movingSpeedLineEdits.resize(NUM_OF_MOTORS);
    for (int i = 0; i < NUM_OF_MOTORS; ++i)
    {
        motorIdLabels[i] = new QLabel(QString::number(i + 1));
        presentPosSliders[i] = new DoubleSlider(Qt::Horizontal, this);
        presentPosSliders[i]->setMinimum(-2606);
        presentPosSliders[i]->setMaximum(2606);
        presentPosSliders[i]->setValue(0);
        presentPosSliders[i]->setSecondValue(0);

        std::ostringstream oss;
        oss.precision(4);
        oss.width(8);
        oss.fill(' ');
        oss.setf(std::ios::fixed, std::ios::floatfield);
        oss << 0.0;
        QString str = QString::fromStdString(oss.str());

        presentPosLineEdits[i] = new QLineEdit(str);
        goalPosLineEdits[i] = new QLineEdit(str);
        presentSpeedLineEdits[i] = new QLineEdit(str);
        movingSpeedLineEdits[i] = new QLineEdit(str);
        int maxLineEditWidth = 80;
        int maxLineEditHeight = 16;
        presentPosLineEdits[i]->setMaximumSize(maxLineEditWidth, maxLineEditHeight);
        goalPosLineEdits[i]->setMaximumSize(maxLineEditWidth, maxLineEditHeight);
        presentSpeedLineEdits[i]->setMaximumSize(maxLineEditWidth, maxLineEditHeight);
        movingSpeedLineEdits[i]->setMaximumSize(maxLineEditWidth, maxLineEditHeight);
//        presentPosLineEdits[i]->setMaximumWidth(maxLineEditWidth);
//        goalPosLineEdits[i]->setMaximumWidth(maxLineEditWidth);
//        presentSpeedLineEdits[i]->setMaximumWidth(maxLineEditWidth);
//        movingSpeedLineEdits[i]->setMaximumWidth(maxLineEditWidth);

        motorIdLabels[i]->setAlignment(Qt::AlignCenter);
        presentPosLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        goalPosLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        presentSpeedLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        movingSpeedLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        row = i + 1;
        col = 0;
        motorFeedbackSubLayout->addWidget(motorIdLabels[i], row, col++);
        ++col;  // For vline
        motorFeedbackSubLayout->addWidget(presentPosSliders[i], row, col++);
        motorFeedbackSubLayout->addWidget(presentPosLineEdits[i], row, col++);
        ++col;  // For vline
        motorFeedbackSubLayout->addWidget(goalPosLineEdits[i], row, col++);
        ++col;  // For vline
        motorFeedbackSubLayout->addWidget(presentSpeedLineEdits[i], row, col++);
        ++col;  // For vline
        motorFeedbackSubLayout->addWidget(movingSpeedLineEdits[i], row, col++);
    }
    motorFeedbackSubLayout->setAlignment(Qt::AlignTop);
    motorFeedbackSubLayout->setColumnStretch(0, 0);
    motorFeedbackSubLayout->setColumnStretch(1, 2);
    motorFeedbackSubLayout->setColumnStretch(2, 1);
    motorFeedbackSubLayout->setColumnStretch(3, 1);
    motorFeedbackSubLayout->setColumnStretch(4, 1);
    // Vertical spacers
    QVector<QFrame*> vlineFrames(4);
    for (int i = 0; i < vlineFrames.size(); ++i)
    {
        vlineFrames[i] = new QFrame;
        vlineFrames[i]->setFrameShape(QFrame::VLine);
    }
    motorFeedbackSubLayout->addWidget(vlineFrames[3], 0, 8, motorFeedbackSubLayout->rowCount(), 1);
    motorFeedbackSubLayout->addWidget(vlineFrames[2], 0, 6, motorFeedbackSubLayout->rowCount(), 1);
    motorFeedbackSubLayout->addWidget(vlineFrames[1], 0, 4, motorFeedbackSubLayout->rowCount(), 1);
    motorFeedbackSubLayout->addWidget(vlineFrames[0], 0, 1, motorFeedbackSubLayout->rowCount(), 1);


    homeAllMotorsButton = new QPushButton("Home all motors");
    setAllMotorTorquesOffButton = new QPushButton("Set all motor torques OFF");
    setAllMotorTorquesOffButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QGridLayout *motorCommandsSubLayout = new QGridLayout;
    row = 0;
    col = 0;
    motorCommandsSubLayout->addWidget(homeAllMotorsButton, row, col++);
    motorCommandsSubLayout->addWidget(setAllMotorTorquesOffButton, row++, col);


    saveAvailablePosesFileButton = new QPushButton("Save available poses file");
    saveQueuedPosesFileButton = new QPushButton("Save queued poses file");
    loadAvailablePosesFileButton = new QPushButton("Load available poses file");
    loadQueuedPosesFileButton = new QPushButton("Load queued poses file");

    QGridLayout* fileIoSubLayout = new QGridLayout;
    row = 0;
    col = 0;
    fileIoSubLayout->addWidget(saveAvailablePosesFileButton, row, col++);
    fileIoSubLayout->addWidget(saveQueuedPosesFileButton, row++, col--);
    fileIoSubLayout->addWidget(loadAvailablePosesFileButton, row, col++);
    fileIoSubLayout->addWidget(loadQueuedPosesFileButton, row, col--);
    fileIoSubLayout->setAlignment(Qt::AlignTop);


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

    disableMotionButtons();

    QGridLayout* rosButtonsSubLayout = new QGridLayout;
    row = 0;
    col = 0;
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
                    static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (2.618 + 2.618) - 2.618;
            robotPose.jointState.velocity[dxlId - 1] =
                    static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (11.8668 + 11.8668) - 11.8668;
            robotPose.jointState.effort[dxlId - 1] =
                    static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (1.0 + 1.0) - 1.0;
        }

        availablePosesList.push_back(robotPose);
    }


    availablePosesCustomListWidget = new CustomListWidget(availablePosesList, "Available poses", 0, this);
    queuedPosesCustomListWidget = new CustomListWidget(queuedPosesList, "Queued poses", 1, this);

    QWidget* motorFeedbackWidget = new QWidget(this);
    motorFeedbackWidget->setLayout(motorFeedbackSubLayout);

    QWidget* rosButtonsWidget = new QWidget(this);
    rosButtonsWidget->setLayout(rosButtonsSubLayout);

    QWidget* poseButtonsWidget = new QWidget(this);
    poseButtonsWidget->setLayout(poseButtonsSubLayout);

    QWidget* motorCommandsWidget =  new QWidget(this);
    motorCommandsWidget->setLayout(motorCommandsSubLayout);

    QWidget* fileIoWidget =  new QWidget(this);
    fileIoWidget->setLayout(fileIoSubLayout);

    outputLog->setReadOnly(true);

        row = 0;
        col = 0;
        QGridLayout* poseControlLayout = new QGridLayout;
//        layout->addWidget(motorFeedbackWidget, row, col++);
        poseControlLayout->addWidget(rosButtonsWidget, row, col++);
        poseControlLayout->addWidget(availablePosesCustomListWidget, row, col++);
        poseControlLayout->addWidget(poseButtonsWidget, row, col++);
        poseControlLayout->addWidget(queuedPosesCustomListWidget, row++, col++);

    //    col = 0;
    //    layout->addWidget(motorCommandsWidget, row, col++);
    //    layout->addWidget(fileIoWidget, row++, col--, 1, 3);
    //    layout->addWidget(outputLog, row++, col, 1, -1);

    QWidget* poseControlWidget =  new QWidget(this);
    poseControlWidget->setLayout(poseControlLayout);


    motorFeedbackDockWidget = new QDockWidget("Motor feedback", this);
    motorFeedbackDockWidget->setWidget(motorFeedbackWidget);
    motorFeedbackDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                         QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    motorCommandsDockWidget = new QDockWidget("Motor commands", this);
    motorCommandsDockWidget->setWidget(motorCommandsWidget);
    motorCommandsDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                         QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    poseControlDockWidget = new QDockWidget("Poses", this);
    poseControlDockWidget->setWidget(poseControlWidget);
    poseControlDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                       QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    fileIoDockWidget = new QDockWidget("File I/O", this);
    fileIoDockWidget->setWidget(fileIoWidget);
    fileIoDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                  QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    outputLogDockWidget = new QDockWidget("Log", this);
    outputLogDockWidget->setWidget(outputLog);
    outputLogDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                     QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    motorValueEditorDockWidget = new QDockWidget("Motor value editor", this);
    motorValueEditorDockWidget->setWidget(motorValueEditor);
    motorValueEditorDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                            QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    motorAddressEditorDockWidget = new QDockWidget("Motor address editor", this);
    motorAddressEditorDockWidget->setWidget(motorAddressEditor);
    motorAddressEditorDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                              QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    motorDialsDockWidget = new QDockWidget("Motor position dials", this);
    motorDialsDockWidget->setWidget(motorDials);
    motorDialsDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                      QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    sensorGrapherDockWidget = new QDockWidget("Sensor Grapher", this);
    sensorGrapherDockWidget->setWidget(sensorGrapher);
    sensorGrapherDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                      QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    setDockNestingEnabled(true);
    addDockWidget(Qt::LeftDockWidgetArea, motorFeedbackDockWidget);
    addDockWidget(Qt::LeftDockWidgetArea, motorCommandsDockWidget);
    addDockWidget(Qt::RightDockWidgetArea, poseControlDockWidget);
    addDockWidget(Qt::RightDockWidgetArea, fileIoDockWidget);
    addDockWidget(Qt::BottomDockWidgetArea, outputLogDockWidget);
    addDockWidget(Qt::LeftDockWidgetArea, sensorGrapherDockWidget);
    tabifyDockWidget(motorFeedbackDockWidget, sensorGrapherDockWidget);
    motorFeedbackDockWidget->raise();

    motorValueEditorDockWidget->setFloating(true);
    motorValueEditorDockWidget->move( QApplication::desktop()->screenGeometry().center() -
                                      motorValueEditorDockWidget->rect().center() );
    motorValueEditorDockWidget->hide();

    motorAddressEditorDockWidget->setFloating(true);
    motorAddressEditorDockWidget->move( QApplication::desktop()->screenGeometry().center() -
                                        motorAddressEditorDockWidget->rect().center() );
    motorAddressEditorDockWidget->hide();

    motorDialsDockWidget->setFloating(true);
    motorDialsDockWidget->move( QApplication::desktop()->screenGeometry().center() -
                                motorDialsDockWidget->rect().center() );
    motorDialsDockWidget->hide();

//    sensorGrapherDockWidget->setFloating(true);
//    sensorGrapherDockWidget->move( QApplication::desktop()->screenGeometry().center() -
//                                sensorGrapherDockWidget->rect().center() );
//    sensorGrapherDockWidget->hide();


    exitAct = new QAction("E&xit", this);
    QMenu* fileMenu = new QMenu;
    fileMenu = menuBar()->addMenu("&File");
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    QMenu* viewMenu = new QMenu;
    viewMenu = menuBar()->addMenu("&View");
    viewMenu->addAction(motorFeedbackDockWidget->toggleViewAction());
    viewMenu->addAction(motorCommandsDockWidget->toggleViewAction());
    viewMenu->addAction(poseControlDockWidget->toggleViewAction());
    viewMenu->addAction(fileIoDockWidget->toggleViewAction());
    viewMenu->addAction(outputLogDockWidget->toggleViewAction());
    viewMenu->addAction(motorValueEditorDockWidget->toggleViewAction());
    viewMenu->addAction(motorAddressEditorDockWidget->toggleViewAction());
    viewMenu->addAction(motorDialsDockWidget->toggleViewAction());
    viewMenu->addAction(sensorGrapherDockWidget->toggleViewAction());

    aboutQtAct = new QAction("About &Qt", this);
    aboutAct = new QAction("&About", this);
    QMenu* helpMenu = new QMenu;
    helpMenu = menuBar()->addMenu("&Help");
    helpMenu->addAction(aboutQtAct);
    helpMenu->addAction(aboutAct);
}


void MainWindow::customiseLayout()
{
    QString mainWindowStyleSheet =
            ( "QMainWindow {"
              "border: 2px solid steelblue;"
              //"background-color: grey;"
              //"border-image: url(assets/images/carbon-fibre-patterns/carbon3.jpg) 0 0 0 0 stretch stretch; }" );
              //"background-image: url(assets/images/carbon-fibre-patterns/carbon3.jpg); }" );
              //"background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
              //"stop: 0 lightsteelblue, stop: 1 steelblue); }" );
              "background-color: qradialgradient(cx:0, cy:0, radius: 1,"
              "fx:0.5, fy:0.5, stop:0 lightgrey, stop:1 grey); }" );

    QString menuBarStyleSheet =
            ( "QMenuBar {"
              "background-color: grey; }" );

    QString dockWidgetStyleSheet =
            ( "QDockWidget {"
//              "titlebar-close-icon: url(assets/images/pixicus/icons/minus_light_alt.png);"
//              "titlebar-normal-icon: url(assets/images/pixicus/icons/export_box.png); }"
//              "titlebar-close-icon: url(assets/images/pp-icon-set/PP Icon Set - PNG Files/Cross.png);"
//              "titlebar-normal-icon: url(assets/images/pp-icon-set/PP Icon Set - PNG Files/Images.png);" );
//              "titlebar-close-icon: url(assets/images/WindowsIcons-master/WindowsPhone/svg/appbar.close.svg);"
//              "titlebar-normal-icon: url(assets/images/WindowsIcons-master/WindowsPhone/svg/appbar.door.leave.svg);"
              //"titlebar-close-icon: url(assets/images/open-iconic-master/svg/x.svg);"
              //"titlebar-normal-icon: url(assets/images/open-iconic-master/svg/external-link.svg);"
              "titlebar-close-icon: url(assets/images/ionicons-2.0.1/src/close.svg);"
              "titlebar-normal-icon: url(assets/images/ionicons-2.0.1/src/android-expand.svg);"
              "border: 2px solid steelblue;"
              //"background-color: grey; }"
              "background: qradialgradient(cx:0, cy:0, radius: 1,"
              "fx:0.5, fy:0.5, stop:0 lightgrey, stop:1 grey); }"
              "QDockWidget::title {"
              "background: steelblue;"
              "padding-right: -200px; }"  // Negative padding stops the vertical title bar from being truncated
              "QDockWidget::close-button, QDockWidget::float-button {"
              "border: 2px solid steelblue;"
              "background: steelblue; }"
              "QDockWidget::close-button:hover, QDockWidget::float-button:hover {"
              "background: lightsteelblue; }"
              "QDockWidget::close-button:pressed, QDockWidget::float-button:pressed {"
              "background: rgb(50, 110, 160); }" );

    QString buttonStyleSheet =
            ( "QPushButton {"
              "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
              "stop: 0 lightsteelblue, stop: 1 steelblue);"
              "border: solid #8F8F91;"
              "border-style: outset;"
              "border-width: 4px;"
              "border-radius: 10px; }"
              //"border-color: beige; }"
              //"font: bold 14px; }"
              //"min-width: 10em;"
              //"padding: 6px; }"
              "QPushButton:flat {"
              "border: none;"  /* no border for a flat push button */
              "}"
              "QPushButton:default {"
              "border-color: navy;"  /* make the default button prominent */
              "}"
              "QPushButton:pressed {"
              "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
              "stop: 0 royalblue, stop: 1 dodgerblue);"
              "border-style: inset; }" );

    QString redButtonStyleSheet =
            ( "QPushButton {"
              "color: white;"
              "background-color: red;"
              "border: solid white;"
              "border-style: outset;"
              "border-width: 4px;"
              "border-radius: 4px; }"
              "QPushButton:flat {"
              "border: none;"  /* no border for a flat push button */
              "}"
              "QPushButton:default {"
              "border-color: grey;"  /* make the default button prominent */
              "}"
              "QPushButton:pressed {"
              "background-color: maroon;"
              "border-style: inset; }" );

    QString editBoxStyleSheet =
            ( "background-color: lightslategrey;" );

    setWindowTitle("ROSoloid Control GUI");
    setWindowIcon( QIcon("assets/images/ionicons-2.0.1/src/gear-a.svg") );
    //setWindowFlags(Qt::CustomizeWindowHint);
    //setWindowFlags(Qt::FramelessWindowHint);

    setStyleSheet(mainWindowStyleSheet);
    menuBar()->setStyleSheet(menuBarStyleSheet);
    //
    saveAvailablePosesFileButton->setStyleSheet(buttonStyleSheet);
    saveQueuedPosesFileButton->setStyleSheet(buttonStyleSheet);
    loadAvailablePosesFileButton->setStyleSheet(buttonStyleSheet);
    loadQueuedPosesFileButton->setStyleSheet(buttonStyleSheet);
    //
    initRosNodeButton->setStyleSheet(buttonStyleSheet);
    initMoveItHandlerButton->setStyleSheet(buttonStyleSheet);
    setCurrentAsStartStateButton->setStyleSheet(buttonStyleSheet);
    setCurrentAsGoalStateButton->setStyleSheet(buttonStyleSheet);
    planMotionButton->setStyleSheet(buttonStyleSheet);
    executeMotionButton->setStyleSheet(buttonStyleSheet);
    //
    addPoseButton->setStyleSheet(buttonStyleSheet);
    removePoseButton->setStyleSheet(buttonStyleSheet);
    addToQueueButton->setStyleSheet(buttonStyleSheet);
    removeFromQueueButton->setStyleSheet(buttonStyleSheet);
    planAndExecuteChainButton->setStyleSheet(buttonStyleSheet);
    testButton->setStyleSheet(buttonStyleSheet);
    //
    for (int i = 0; i < NUM_OF_MOTORS; ++i)
    {
        presentPosSliders[i]->setStyleSheet(editBoxStyleSheet);
        presentPosLineEdits[i]->setStyleSheet(editBoxStyleSheet);
        goalPosLineEdits[i]->setStyleSheet(editBoxStyleSheet);
        presentSpeedLineEdits[i]->setStyleSheet(editBoxStyleSheet);
        movingSpeedLineEdits[i]->setStyleSheet(editBoxStyleSheet);
    }
    //
    homeAllMotorsButton->setStyleSheet(buttonStyleSheet);
    setAllMotorTorquesOffButton->setStyleSheet(redButtonStyleSheet);
    //
    outputLog->setStyleSheet(editBoxStyleSheet);
    //
    motorFeedbackDockWidget->setStyleSheet(dockWidgetStyleSheet);
    motorCommandsDockWidget->setStyleSheet(dockWidgetStyleSheet);
    poseControlDockWidget->setStyleSheet(dockWidgetStyleSheet);
    fileIoDockWidget->setStyleSheet(dockWidgetStyleSheet);
    outputLogDockWidget->setStyleSheet(dockWidgetStyleSheet);
    motorValueEditorDockWidget->setStyleSheet(dockWidgetStyleSheet);
    motorAddressEditorDockWidget->setStyleSheet(dockWidgetStyleSheet);
    motorDialsDockWidget->setStyleSheet(dockWidgetStyleSheet);
    sensorGrapherDockWidget->setStyleSheet(dockWidgetStyleSheet);
}


void MainWindow::connectSignalsAndSlots()
{
    connect( initRosNodeButton, SIGNAL(clicked()), this, SLOT(initRosNode()) );
    connect( initMoveItHandlerButton, SIGNAL(clicked()), this, SLOT(initMoveItHandler()) );
    connect( addPoseButton, SIGNAL(clicked()), this, SLOT(addPose()) );
    connect( removePoseButton, SIGNAL(clicked()), this, SLOT(removePose()) );

    connect( setCurrentAsStartStateButton, SIGNAL(clicked()), moveItHandler, SLOT(setCurrentAsStartState()) );
    connect( setCurrentAsGoalStateButton, SIGNAL(clicked()), moveItHandler, SLOT(setCurrentAsGoalState()) );
    connect( planMotionButton, SIGNAL(clicked()), moveItHandler, SLOT(planMotion()) );
    connect( executeMotionButton, SIGNAL(clicked()), moveItHandler, SLOT(executeMotion()) );
    connect( planAndExecuteChainButton, SIGNAL(clicked()), this, SLOT(planAndExecuteChain()) );

    connect( addToQueueButton, SIGNAL(clicked()), this, SLOT(addToQueue()) );
    connect( removeFromQueueButton, SIGNAL(clicked()), this, SLOT(removeFromQueue()) );

    connect( availablePosesCustomListWidget->getListView(), SIGNAL(clicked(const QModelIndex &)),
             this, SLOT(updateJointStateValuesFromPose(const QModelIndex &)) );
    connect( availablePosesCustomListWidget->getListView(), SIGNAL(activated(const QModelIndex &)),
             this, SLOT(updateJointStateValuesFromPose(const QModelIndex &)) );

    connect( saveAvailablePosesFileButton, SIGNAL(clicked()), availablePosesCustomListWidget, SLOT(savePosesFile()) );
    connect( saveQueuedPosesFileButton, SIGNAL(clicked()), queuedPosesCustomListWidget, SLOT(savePosesFile()) );
    connect( loadAvailablePosesFileButton, SIGNAL(clicked()), availablePosesCustomListWidget, SLOT(loadPosesFile()) );
    connect( loadQueuedPosesFileButton, SIGNAL(clicked()), queuedPosesCustomListWidget, SLOT(loadPosesFile()) );

    connect( rosWorker, SIGNAL(connectedToRosMaster()), this, SLOT(nodeConnectedToRosMaster()) );
    connect( rosWorker, SIGNAL(connectedToRosMaster()), this, SLOT(enableMotionButtons()) );
    connect( rosWorker, SIGNAL(disconnectedFromRosMaster()), this, SLOT(nodeDisconnectedFromRosMaster()) );
    connect( rosWorker, SIGNAL(disconnectedFromRosMaster()), this, SLOT(disableMotionButtons()) );
    connect( rosWorker, SIGNAL(jointStateUpdated(sensor_msgs::JointState)),
             this, SLOT(updateJointStateValues(sensor_msgs::JointState)) );
    connect( rosWorker, SIGNAL(secondaryDataUpdated(sensor_msgs::JointState)),
             this, SLOT(updateSecondaryRobotValues(sensor_msgs::JointState)) );

    connect( homeAllMotorsButton, SIGNAL(clicked()), rosWorker, SLOT(homeAllMotors()) );
    connect( setAllMotorTorquesOffButton, SIGNAL(clicked()), rosWorker, SLOT(setAllMotorTorquesOff()) );

    connect( motorDialsDockWidget, SIGNAL(visibilityChanged(bool)), motorDials, SLOT(initialiseDials(bool)) );

    connect( aboutQtAct, SIGNAL(triggered()), this, SLOT(aboutQt()) );
    connect( aboutAct, SIGNAL(triggered()), this, SLOT(about()) );
    connect( exitAct, SIGNAL(triggered()), this, SLOT(quit()) );
}


void MainWindow::initRosNode()
{
    rosWorker->init();
    outputLog->appendTimestamped("ROS node initialised");
}


void MainWindow::initMoveItHandler()
{
    moveItHandler->init();
    outputLog->appendTimestamped("MoveIt! handler initialised");
}


void MainWindow::addPose()
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
            robotPose.jointState = rosWorker->getCurrentJointState();
            robotPose.dwellTimeInSec = inputTime;
            availablePosesCustomListWidget->add(robotPose);
        }
        else QMessageBox::warning(this, "Pose not added",
                                  "Incorrect pose dwell time.");
    }
    else QMessageBox::warning(this, "Pose not added",
                              "Incorrect pose name.");

}


void MainWindow::removePose()
{
    availablePosesCustomListWidget->remove();
}


void MainWindow::planAndExecuteChain()
{
    //moveItHandler->planAndExecuteChain(queuedPosesCustomListWidget->getRobotPosesListModel()->getRobotPosesList());


    // Thread test
    QList<RobotPose> robotPosesList = queuedPosesCustomListWidget->getRobotPosesListModel()->getRobotPosesList();

    workerThread = new QThread;
    PlanAndExecuteChainWorker* planAndExecuteChainWorker =
            new PlanAndExecuteChainWorker(robotPosesList, rosWorker, &moveMutex);
    planAndExecuteChainWorker->moveToThread(workerThread);
    connect( workerThread, SIGNAL(started()), planAndExecuteChainWorker, SLOT(doWork()) );
    connect( planAndExecuteChainWorker, SIGNAL(finished()), workerThread, SLOT(quit()) );
    connect( planAndExecuteChainWorker, SIGNAL(finished()), planAndExecuteChainWorker, SLOT(deleteLater()) );
    connect( workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()) );
    workerThread->start();
}


void MainWindow::addToQueue()
{
    queuedPosesCustomListWidget->addFrom(availablePosesCustomListWidget);
}


void MainWindow::removeFromQueue()
{
    queuedPosesCustomListWidget->remove();
}


void MainWindow::nodeConnectedToRosMaster()
{
    outputLog->appendTimestamped("ROS node connected to master");
}


void MainWindow::nodeDisconnectedFromRosMaster()
{
    outputLog->appendTimestamped("ROS node disconnected from master");
}


void MainWindow::enableMotionButtons()
{
    initMoveItHandlerButton->setEnabled(true);
    addPoseButton->setEnabled(true);
    setCurrentAsStartStateButton->setEnabled(true);
    setCurrentAsGoalStateButton->setEnabled(true);
    planMotionButton->setEnabled(true);
    executeMotionButton->setEnabled(true);
    planAndExecuteChainButton->setEnabled(true);
}


void MainWindow::disableMotionButtons()
{
    initMoveItHandlerButton->setEnabled(false);
    addPoseButton->setEnabled(false);
    setCurrentAsStartStateButton->setEnabled(false);
    setCurrentAsGoalStateButton->setEnabled(false);
    planMotionButton->setEnabled(false);
    executeMotionButton->setEnabled(false);
    planAndExecuteChainButton->setEnabled(false);
}


void MainWindow::updateJointStateValues(sensor_msgs::JointState js)
{
    if ( (js.position.size() >= NUM_OF_MOTORS) && (js.velocity.size() >= NUM_OF_MOTORS) )
    {
        for (int i = 0; i < NUM_OF_MOTORS; ++i)
        {
            // -2.606..2.606 rad, converted to -2606..2606 int range
            presentPosSliders[i]->setFirstValue(js.position[i] * 1000);

            std::ostringstream oss;
            oss.precision(4);
            oss.width(8);
            oss.fill(' ');
            oss.setf(std::ios::fixed, std::ios::floatfield);
            oss << js.position[i];
            presentPosLineEdits[i]->setText( QString::fromStdString(oss.str()) );

            oss.str("");
            oss.width(8);  // Not 'sticky'
            oss << js.velocity[i];
            presentSpeedLineEdits[i]->setText( QString::fromStdString(oss.str()) );
        }
    }
}


void MainWindow::updateSecondaryRobotValues(sensor_msgs::JointState js)
{
    if ( (js.position.size() >= NUM_OF_MOTORS) && (js.velocity.size() >= NUM_OF_MOTORS) )
    {
        for (int i = 0; i < NUM_OF_MOTORS; ++i)
        {
            std::ostringstream oss;
            oss.precision(4);
            oss.width(8);
            oss.fill(' ');
            oss.setf(std::ios::fixed, std::ios::floatfield);
            oss << js.position[i];
            goalPosLineEdits[i]->setText( QString::fromStdString(oss.str()) );

            oss.str("");
            oss.width(8);  // Not 'sticky'
            oss << js.velocity[i];
            movingSpeedLineEdits[i]->setText( QString::fromStdString(oss.str()) );
        }
    }
}


void MainWindow::updateJointStateValuesFromPose(const QModelIndex &modelIndex)
{
    sensor_msgs::JointState js = availablePosesCustomListWidget->
            getRobotPosesListModel()->getCurrentPose(modelIndex).jointState;
    if (js.position.size() >= NUM_OF_MOTORS)
    {
    for (int i = 0; i < NUM_OF_MOTORS; ++i)
        presentPosSliders[i]->setSecondValue(js.position[i] * 1000);
    }
}


void MainWindow::aboutQt()
{
    QMessageBox::aboutQt(this);
}


void MainWindow::about()
{
   QMessageBox::about(this, "About ROSoloid Control GUI",
                      "This GUI interacts with a ROS-based AX-12+ servo controller and the ROS MoveIt! API.\n"
                      "\n"
                      "Author: D. Xydas\n"
                      "e-mail: dxydas7@gmail.com" );
}


void MainWindow::quit()
{
    QApplication::quit();
}
