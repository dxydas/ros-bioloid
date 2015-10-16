#include "mainwindow.h"
#include <iostream>
#include <sstream>
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QString>
#include <qt5/QtCore/QMetaType>
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

#define NUM_OF_MOTORS 18

Q_DECLARE_METATYPE(sensor_msgs::JointState)


MainWindow::MainWindow(int argc, char* argv[], QWidget* parent) :
    QMainWindow(parent)
{
    rosWorker = new RosWorker(argc, argv, "robot_gui");
    motorValueEditor = new MotorValueEditor(rosWorker);
    motorDials = new MotorDials(rosWorker);

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
    QLabel* presentPositionLabel = new QLabel("Present\nposition");
    QLabel* goalPositionLabel = new QLabel("Goal\nposition");
    QLabel* presentSpeedLabel = new QLabel("Present\nspeed");
    QLabel* movingSpeedLabel = new QLabel("Moving\nspeed");

    motorLabel->setAlignment(Qt::AlignCenter);
    presentPositionLabel->setAlignment(Qt::AlignCenter);
    goalPositionLabel->setAlignment(Qt::AlignCenter);
    presentSpeedLabel->setAlignment(Qt::AlignCenter);
    movingSpeedLabel->setAlignment(Qt::AlignCenter);

    presentPositionLabel->setStyleSheet("QLabel { color: royalblue }");
    goalPositionLabel->setStyleSheet("QLabel { color: midnightblue }");
    presentSpeedLabel->setStyleSheet("QLabel { color: royalblue }");
    movingSpeedLabel->setStyleSheet("QLabel { color: midnightblue }");

    QGridLayout* motorFeedbackSubLayout = new QGridLayout;
    motorFeedbackSubLayout->addWidget(motorLabel, row, col++);
    ++col;  // Leave a blank column in order to add vline spacer later
    motorFeedbackSubLayout->addWidget(presentPositionLabel, row, col++, 1, 2);
    ++col;  // presentPositionLabel covers two columns
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
        presentPosSliders[i]->setMinimum(-2618);
        presentPosSliders[i]->setMaximum(2618);
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


    setAllMotorTorquesOffButton = new QPushButton("Set all motor torques OFF");
    setAllMotorTorquesOffButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setAllMotorTorquesOffButton->setStyleSheet("QPushButton { color: white; background-color: #AA0000 }");

    QGridLayout *motorCommandsSubLayout = new QGridLayout;
    row = 0;
    col = 0;
    motorCommandsSubLayout->addWidget(setAllMotorTorquesOffButton, row, col++, -1, 1);


    saveAvailablePosesFileButton = new QPushButton("Save available poses file");
    loadAvailablePosesFileButton = new QPushButton("Load available poses file");
    saveQueuedPosesFileButton = new QPushButton("Save queued poses file");
    loadQueuedPosesFileButton = new QPushButton("Load queued poses file");

    QGridLayout* fileIoSubLayout = new QGridLayout;
    row = 0;
    col = 0;
    fileIoSubLayout->addWidget(saveAvailablePosesFileButton, row, col++);
    fileIoSubLayout->addWidget(loadAvailablePosesFileButton, row++, col--);
    fileIoSubLayout->addWidget(saveQueuedPosesFileButton, row, col++);
    fileIoSubLayout->addWidget(loadQueuedPosesFileButton, row, col--);


    initRosNodeButton = new QPushButton("Initialise ROS node");
    addPoseButton = new QPushButton("Add pose");
    removePoseButton = new QPushButton("Remove pose");
    setStartStateButton = new QPushButton("Set as start state");
    setGoalStateButton = new QPushButton("Set as goal state");
    planMotionButton = new QPushButton("Run motion planner");
    executeMotionButton = new QPushButton("Move to pose");
    addToQueueButton = new QPushButton("Add to queue -->");
    removeFromQueueButton = new QPushButton("<-- Remove from queue");

    disableMotionButtons();

    QGridLayout* poseButtonsSubLayout = new QGridLayout;
    row = 0;
    col = 0;
    poseButtonsSubLayout->addWidget(initRosNodeButton, row++, col);
    poseButtonsSubLayout->addWidget(addPoseButton, row++, col);
    poseButtonsSubLayout->addWidget(removePoseButton, row++, col);
    poseButtonsSubLayout->addWidget(setStartStateButton, row++, col);
    poseButtonsSubLayout->addWidget(setGoalStateButton, row++, col);
    poseButtonsSubLayout->addWidget(planMotionButton, row++, col);
    poseButtonsSubLayout->addWidget(executeMotionButton, row++, col);
    poseButtonsSubLayout->addWidget(addToQueueButton, row++, col);
    poseButtonsSubLayout->addWidget(removeFromQueueButton, row++, col);


    QList<RobotPoseStruct> availablePosesList;
    QList<RobotPoseStruct> queuedPosesList;

    RobotPoseStruct poseStruct;
    for (int i = 0; i < 5; ++i)
    {
        std::ostringstream oss;
        oss.width(3);
        oss.fill('0');
        oss.setf(std::ios::fixed, std::ios::floatfield);
        oss << i;
        poseStruct.name = "test" + QString::fromStdString(oss.str());

        for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
        {
            poseStruct.jointState.position[dxlId - 1] =
                    static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (2.618 + 2.618) - 2.618;
            poseStruct.jointState.velocity[dxlId - 1] =
                    static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (11.8668 + 11.8668) - 11.8668;
            poseStruct.jointState.effort[dxlId - 1] =
                    static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (1.0 + 1.0) - 1.0;
        }

        availablePosesList.push_back(poseStruct);
    }


    availablePosesCustomListWidget = new CustomListWidget(availablePosesList, "Poses available", 0, this);
    queuedPosesCustomListWidget = new CustomListWidget(queuedPosesList, "Poses queued", 1, this);

    QWidget* motorFeedbackWidget = new QWidget(this);
    motorFeedbackWidget->setLayout(motorFeedbackSubLayout);

    QWidget* poseButtonsWidget = new QWidget(this);
    poseButtonsWidget->setLayout(poseButtonsSubLayout);

    QWidget* motorCommandsWidget =  new QWidget(this);
    motorCommandsWidget->setLayout(motorCommandsSubLayout);

    QWidget* fileIoWidget =  new QWidget(this);
    fileIoWidget->setLayout(fileIoSubLayout);

    outputLog = new OutputLog;
    outputLog->setReadOnly(true);

        row = 0;
        col = 0;
        QGridLayout* poseControlLayout = new QGridLayout;
//        layout->addWidget(motorFeedbackWidget, row, col++);
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

    motorDialsDockWidget = new QDockWidget("Motor position dials", this);
    motorDialsDockWidget->setWidget(motorDials);
    motorDialsDockWidget->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable |
                                      QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetVerticalTitleBar);

    setDockNestingEnabled(true);
    addDockWidget(Qt::LeftDockWidgetArea, motorFeedbackDockWidget);
    addDockWidget(Qt::LeftDockWidgetArea, motorCommandsDockWidget);
    addDockWidget(Qt::RightDockWidgetArea, poseControlDockWidget);
    addDockWidget(Qt::RightDockWidgetArea, fileIoDockWidget);
    addDockWidget(Qt::BottomDockWidgetArea, outputLogDockWidget);

    motorValueEditorDockWidget->setFloating(true);
    motorValueEditorDockWidget->move( QApplication::desktop()->screenGeometry().center() -
                                      motorValueEditorDockWidget->rect().center() );
    motorValueEditorDockWidget->hide();

    motorDialsDockWidget->setFloating(true);
    motorDialsDockWidget->move( QApplication::desktop()->screenGeometry().center() -
                                motorDialsDockWidget->rect().center() );
    motorDialsDockWidget->hide();


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
    viewMenu->addAction(motorDialsDockWidget->toggleViewAction());

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
    loadAvailablePosesFileButton->setStyleSheet(buttonStyleSheet);
    saveQueuedPosesFileButton->setStyleSheet(buttonStyleSheet);
    loadQueuedPosesFileButton->setStyleSheet(buttonStyleSheet);
    //
    initRosNodeButton->setStyleSheet(buttonStyleSheet);
    addPoseButton->setStyleSheet(buttonStyleSheet);
    removePoseButton->setStyleSheet(buttonStyleSheet);
    setStartStateButton->setStyleSheet(buttonStyleSheet);
    setGoalStateButton->setStyleSheet(buttonStyleSheet);
    planMotionButton->setStyleSheet(buttonStyleSheet);
    executeMotionButton->setStyleSheet(buttonStyleSheet);
    addToQueueButton->setStyleSheet(buttonStyleSheet);
    removeFromQueueButton->setStyleSheet(buttonStyleSheet);
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
    outputLog->setStyleSheet(editBoxStyleSheet);
    //
    motorFeedbackDockWidget->setStyleSheet(dockWidgetStyleSheet);
    motorCommandsDockWidget->setStyleSheet(dockWidgetStyleSheet);
    poseControlDockWidget->setStyleSheet(dockWidgetStyleSheet);
    fileIoDockWidget->setStyleSheet(dockWidgetStyleSheet);
    outputLogDockWidget->setStyleSheet(dockWidgetStyleSheet);
    motorValueEditorDockWidget->setStyleSheet(dockWidgetStyleSheet);
    motorDialsDockWidget->setStyleSheet(dockWidgetStyleSheet);
}


void MainWindow::connectSignalsAndSlots()
{
    connect( initRosNodeButton, SIGNAL(clicked()), this, SLOT(initRosNode()) );
    connect( addPoseButton, SIGNAL(clicked()), this, SLOT(addPose()) );
    connect( removePoseButton, SIGNAL(clicked()), this, SLOT(removePose()) );
    connect( setStartStateButton, SIGNAL(clicked()), this, SLOT(setStartState()) );
    connect( setGoalStateButton, SIGNAL(clicked()), this, SLOT(setGoalState()) );
    connect( planMotionButton, SIGNAL(clicked()), this, SLOT(planMotion()) );
    connect( executeMotionButton, SIGNAL(clicked()), this, SLOT(executeMotion()) );
    connect( addToQueueButton, SIGNAL(clicked()), this, SLOT(addToQueue()) );
    connect( removeFromQueueButton, SIGNAL(clicked()), this, SLOT(removeFromQueue()) );

    connect( availablePosesCustomListWidget->getListView(), SIGNAL(clicked(const QModelIndex &)),
             this, SLOT(updateJointStateValuesFromPose(const QModelIndex &)) );
    connect( availablePosesCustomListWidget->getListView(), SIGNAL(activated(const QModelIndex &)),
             this, SLOT(updateJointStateValuesFromPose(const QModelIndex &)) );

    connect( saveAvailablePosesFileButton, SIGNAL(clicked()), availablePosesCustomListWidget, SLOT(savePosesFile()) );
    connect( loadAvailablePosesFileButton, SIGNAL(clicked()), availablePosesCustomListWidget, SLOT(loadPosesFile()) );
    connect( saveQueuedPosesFileButton, SIGNAL(clicked()), queuedPosesCustomListWidget, SLOT(savePosesFile()) );
    connect( loadQueuedPosesFileButton, SIGNAL(clicked()), queuedPosesCustomListWidget, SLOT(loadPosesFile()) );

    connect( rosWorker, SIGNAL(connectedToRosMaster()), this, SLOT(nodeConnectedToRosMaster()) );
    connect( rosWorker, SIGNAL(connectedToRosMaster()), this, SLOT(enableMotionButtons()) );
    connect( rosWorker, SIGNAL(disconnectedFromRosMaster()), this, SLOT(nodeDisconnectedFromRosMaster()) );
    connect( rosWorker, SIGNAL(disconnectedFromRosMaster()), this, SLOT(disableMotionButtons()) );
    qRegisterMetaType<sensor_msgs::JointState>("JointState");
    connect( rosWorker, SIGNAL(jointStateUpdated(sensor_msgs::JointState)),
             this, SLOT(updateJointStateValues(sensor_msgs::JointState)) );
    connect( rosWorker, SIGNAL(secondaryDataUpdated(sensor_msgs::JointState)),
             this, SLOT(updateSecondaryRobotValues(sensor_msgs::JointState)) );

    connect( setAllMotorTorquesOffButton, SIGNAL(clicked()), rosWorker, SLOT(setAllMotorTorquesOff()) );

    connect( motorDialsDockWidget, SIGNAL(visibilityChanged(bool)), motorDials, SLOT(initialiseDials(bool)) );

    connect( aboutQtAct, SIGNAL(triggered()), qApp, SLOT(aboutQt()) );
    connect( aboutAct, SIGNAL(triggered()), this, SLOT(about()) );
    connect( exitAct, SIGNAL(triggered()), this, SLOT(quit()) );
}


void MainWindow::initRosNode()
{
    outputLog->appendTimestamped("ROS node initialised");
    rosWorker->init();
}


void MainWindow::addPose()
{
    bool ok;
    QString text = QInputDialog::getText(this, "New pose",
                                         "Enter new pose name:", QLineEdit::Normal,
                                         "NewPose001", &ok);
    if (ok && !text.isEmpty())
    {
        RobotPoseStruct poseStruct;
        poseStruct.name = text;

        poseStruct.jointState = rosWorker->getCurrentJointState();
        availablePosesCustomListWidget->add(poseStruct);
    }
}


void MainWindow::removePose()
{
    availablePosesCustomListWidget->remove();
}


void MainWindow::setStartState()
{

}


void MainWindow::setGoalState()
{

}


void MainWindow::planMotion()
{

}


void MainWindow::executeMotion()
{

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
    addPoseButton->setEnabled(true);
    setStartStateButton->setEnabled(true);
    setGoalStateButton->setEnabled(true);
    planMotionButton->setEnabled(true);
    executeMotionButton->setEnabled(true);
}


void MainWindow::disableMotionButtons()
{
    addPoseButton->setEnabled(false);
    setStartStateButton->setEnabled(false);
    setGoalStateButton->setEnabled(false);
    planMotionButton->setEnabled(false);
    executeMotionButton->setEnabled(false);
}


void MainWindow::updateJointStateValues(sensor_msgs::JointState js)
{
    if ( (js.position.size() >= NUM_OF_MOTORS) && (js.velocity.size() >= NUM_OF_MOTORS) )
    {
        for (int i = 0; i < NUM_OF_MOTORS; ++i)
        {
            // -2.618..2.618 rad, converted to -2618..2618 int range
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

