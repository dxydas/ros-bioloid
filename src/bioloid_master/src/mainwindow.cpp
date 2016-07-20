#include "mainwindow.h"
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QMetaType>
#include <qt5/QtCore/QString>
#include <qt5/QtCore/QFile>
#include <qt5/QtGui/QIcon>
#include <qt5/QtWidgets/QApplication>
#include <qt5/QtWidgets/QMenuBar>
#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QDesktopWidget>
#include <qt5/QtWidgets/QMessageBox>

Q_DECLARE_METATYPE(sensor_msgs::JointState)
Q_DECLARE_METATYPE(geometry_msgs::Vector3)
Q_DECLARE_METATYPE(std_msgs::Float32)
Q_DECLARE_METATYPE(std_msgs::Int16MultiArray)


MainWindow::MainWindow(int argc, char* argv[], QWidget* parent) :
    QMainWindow(parent)
{
    setWindowTitle("ROSoloid Control GUI");

    qRegisterMetaType<sensor_msgs::JointState>("JointState");
    qRegisterMetaType<geometry_msgs::Vector3>("Vector3");
    qRegisterMetaType<std_msgs::Float32>("Float32");
    qRegisterMetaType<std_msgs::Int16MultiArray>("Int16MultiArray");

    outputLog = new OutputLog(this);

    rosWorker = new RosWorker(argc, argv, "rosoloid_gui", outputLog, this);
    moveItHandler = new MoveItHandler(outputLog, this);

    robotController = new RobotController(rosWorker, this);
    motorCommandsWidget = new MotorCommandsWidget(this);
    motorFeedbackWidget = new MotorFeedbackWidget(this);
    fileIoController = new FileIoController(this);
    motorValueEditor = new MotorValueEditor(rosWorker, this);
    motorAddressEditor = new MotorAddressEditor(rosWorker, this);
    motorDials = new MotorDials(rosWorker, this);
    sensorGrapher = new SensorGrapher(rosWorker, this);
    pidBalancerWidget = new PidBalancerWidget(rosWorker, this);

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
    motorFeedbackDockWidget = new QDockWidget("Motor feedback", this);
    motorFeedbackDockWidget->setWidget(motorFeedbackWidget);
    motorFeedbackDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                         QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    motorCommandsDockWidget = new QDockWidget("Motor commands", this);
    motorCommandsDockWidget->setWidget(motorCommandsWidget);
    motorCommandsDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                         QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    poseControlDockWidget = new QDockWidget("Poses", this);
    poseControlDockWidget->setWidget(robotController);
    poseControlDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                       QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    fileIoDockWidget = new QDockWidget("File I/O", this);
    fileIoDockWidget->setWidget(fileIoController);
    fileIoDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                  QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    outputLogDockWidget = new QDockWidget("Log", this);
    outputLogDockWidget->setWidget(outputLog);
    outputLogDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                     QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    motorValueEditorDockWidget = new QDockWidget("Motor value editor", this);
    motorValueEditorDockWidget->setWidget(motorValueEditor);
    motorValueEditorDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                            QDockWidget::DockWidgetFloatable |
                                            QDockWidget::DockWidgetVerticalTitleBar);

    motorAddressEditorDockWidget = new QDockWidget("Motor address editor", this);
    motorAddressEditorDockWidget->setWidget(motorAddressEditor);
    motorAddressEditorDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                              QDockWidget::DockWidgetFloatable |
                                              QDockWidget::DockWidgetVerticalTitleBar);

    motorDialsDockWidget = new QDockWidget("Motor position dials", this);
    motorDialsDockWidget->setWidget(motorDials);
    motorDialsDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                      QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    sensorGrapherDockWidget = new QDockWidget("Sensor grapher", this);
    sensorGrapherDockWidget->setWidget(sensorGrapher);
    sensorGrapherDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                         QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    pidBalancerDockWidget = new QDockWidget("PID balancer", this);
    pidBalancerDockWidget->setWidget(pidBalancerWidget);
    pidBalancerDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
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

    pidBalancerDockWidget->setFloating(true);
    pidBalancerDockWidget->move( QApplication::desktop()->screenGeometry().center() -
                                pidBalancerDockWidget->rect().center() );
    pidBalancerDockWidget->hide();

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
    viewMenu->addAction(pidBalancerDockWidget->toggleViewAction());

    aboutQtAct = new QAction("About &Qt", this);
    aboutAct = new QAction("&About", this);
    QMenu* helpMenu = new QMenu;
    helpMenu = menuBar()->addMenu("&Help");
    helpMenu->addAction(aboutQtAct);
    helpMenu->addAction(aboutAct);
}


void MainWindow::customiseLayout()
{
    QFile file;
    QString mainWindowStyleSheet;

    file.setFileName("assets/qss/rosoloidgui.qss");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    mainWindowStyleSheet.append( QLatin1String(file.readAll()) );
    file.close();

    setStyleSheet(mainWindowStyleSheet);
}


void MainWindow::connectSignalsAndSlots()
{
    connect( robotController->initRosNodeButton, SIGNAL(clicked()), rosWorker, SLOT(initialise()) );
    connect( robotController->initMoveItHandlerButton, SIGNAL(clicked()), moveItHandler, SLOT(initialise()) );
    connect( robotController->addPoseButton, SIGNAL(clicked()), robotController, SLOT(addPose()) );
    connect( robotController->removePoseButton, SIGNAL(clicked()), robotController, SLOT(removePose()) );

    connect( robotController->setCurrentAsStartStateButton, SIGNAL(clicked()),
             moveItHandler, SLOT(setCurrentAsStartState()) );
    connect( robotController->setCurrentAsGoalStateButton, SIGNAL(clicked()),
             moveItHandler, SLOT(setCurrentAsGoalState()) );
    connect( robotController->planMotionButton, SIGNAL(clicked()), moveItHandler, SLOT(planMotion()) );
    connect( robotController->executeMotionButton, SIGNAL(clicked()), moveItHandler, SLOT(executeMotion()) );
    connect( robotController->planAndExecuteChainButton, SIGNAL(clicked()),
             robotController, SLOT(planAndExecuteChain()) );

    connect( robotController->addToQueueButton, SIGNAL(clicked()), robotController, SLOT(addToQueue()) );
    connect( robotController->removeFromQueueButton, SIGNAL(clicked()), robotController, SLOT(removeFromQueue()) );

    connect( robotController->availablePosesCustomListWidget->getListView(), SIGNAL(clicked(const QModelIndex &)),
             robotController, SLOT(updateJointStateValuesFromPoseHelper(const QModelIndex &)) );
    connect( robotController->availablePosesCustomListWidget->getListView(), SIGNAL(activated(const QModelIndex &)),
             robotController, SLOT(updateJointStateValuesFromPoseHelper(const QModelIndex &)) );
    connect( robotController, SIGNAL(jointStateValuesFromPoseReady(sensor_msgs::JointState)),
             motorFeedbackWidget, SLOT(updateJointStateValuesFromPose(sensor_msgs::JointState)) );

    connect( fileIoController->saveAvailablePosesFileButton, SIGNAL(clicked()),
             robotController->availablePosesCustomListWidget, SLOT(savePosesFile()) );
    connect( fileIoController->saveQueuedPosesFileButton, SIGNAL(clicked()),
             robotController->queuedPosesCustomListWidget, SLOT(savePosesFile()) );
    connect( fileIoController->loadAvailablePosesFileButton, SIGNAL(clicked()),
             robotController->availablePosesCustomListWidget, SLOT(loadPosesFile()) );
    connect( fileIoController->loadQueuedPosesFileButton, SIGNAL(clicked()),
             robotController->queuedPosesCustomListWidget, SLOT(loadPosesFile()) );

    connect( rosWorker, SIGNAL(initialised()), robotController, SLOT(nodeInitialised()) );
    connect( rosWorker, SIGNAL(terminated()), robotController, SLOT(nodeTerminated()) );
    connect( rosWorker, SIGNAL(connectedToRosMaster()), robotController, SLOT(nodeConnectedToRosMaster()) );
    connect( rosWorker, SIGNAL(disconnectedFromRosMaster()), robotController, SLOT(nodeDisconnectedFromRosMaster()) );

    connect( rosWorker, SIGNAL(jointStateUpdated(sensor_msgs::JointState)),
             motorFeedbackWidget, SLOT(updateJointStateValues(sensor_msgs::JointState)) );
    connect( rosWorker, SIGNAL(secondaryDataUpdated(sensor_msgs::JointState)),
             motorFeedbackWidget, SLOT(updateSecondaryRobotValues(sensor_msgs::JointState)) );

    connect( motorCommandsWidget->homeAllMotorsButton, SIGNAL(clicked()), rosWorker, SLOT(homeAllMotors()) );
    connect( motorCommandsWidget->setAllMotorTorquesOffButton, SIGNAL(clicked()),
             rosWorker, SLOT(setAllMotorTorquesOff()) );

    connect( motorDialsDockWidget, SIGNAL(visibilityChanged(bool)), motorDials, SLOT(initialiseDials(bool)) );

    connect( aboutQtAct, SIGNAL(triggered()), this, SLOT(aboutQt()) );
    connect( aboutAct, SIGNAL(triggered()), this, SLOT(about()) );
    connect( exitAct, SIGNAL(triggered()), this, SLOT(quit()) );
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
