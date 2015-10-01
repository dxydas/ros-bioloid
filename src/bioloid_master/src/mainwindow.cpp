#include "mainwindow.h"
#include <iostream>
#include <sstream>
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QString>
#include <qt5/QtCore/QMetaType>
#include <qt5/QtWidgets/QApplication>
#include <qt5/QtWidgets/QMenuBar>
#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QFormLayout>
#include <qt5/QtWidgets/QFrame>
#include <qt5/QtWidgets/QInputDialog>
#include <qt5/QtWidgets/QDoubleSpinBox>
#include <qt5/QtWidgets/QAbstractButton>
#include <qt5/QtWidgets/QAbstractSlider>
#include <qt5/QtWidgets/QSizePolicy>
#include <qt5/QtWidgets/QDesktopWidget>

#define NUM_OF_MOTORS 18

Q_DECLARE_METATYPE(sensor_msgs::JointState)


MainWindow::MainWindow(int argc, char* argv[], QWidget* parent) :
    QMainWindow(parent)
{
    setUpLayout();
    outputLog->appendTimestamped("Application started");

    rosWorker = new RosWorker(argc, argv, "robot_gui");

    motorValueEditor = new MotorValueEditor(rosWorker);

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

    connect( openMotorValueEditorButton, SIGNAL(clicked()), this, SLOT(openMotorValueEditor()) );
    connect( setAllMotorTorquesOffButton, SIGNAL(clicked()), rosWorker, SLOT(setAllMotorTorquesOff()) );

    connect( exitAct, SIGNAL(triggered()), this, SLOT(quit()) );
}


MainWindow::~MainWindow()
{
}


void MainWindow::setUpLayout()
{
    setWindowTitle("ROSoloid Control GUI");

    QMenu* fileMenu = new QMenu;
    exitAct = new QAction("E&xit", this);

    fileMenu = menuBar()->addMenu("&File");
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);


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

    presentPositionLabel->setStyleSheet("QLabel { color: blue }");
    goalPositionLabel->setStyleSheet("QLabel { color: green }");
    presentSpeedLabel->setStyleSheet("QLabel { color: blue }");
    movingSpeedLabel->setStyleSheet("QLabel { color: green }");

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
        presentPosSliders[i]->setMinimum(-2700);
        presentPosSliders[i]->setMaximum(2700);
        presentPosSliders[i]->setValue(0);
        presentPosSliders[i]->setSecondValue(0);
        //presentPosSliders[i]->setTickPosition(QSlider::TicksBothSides);
        //presentPosSliders[i]->setTickInterval(200);

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


//    QDoubleSpinBox* setAllMotorSpeedsSpinBox;
//    setAllMotorSpeedsSpinBox = new QDoubleSpinBox;
//    setAllMotorSpeedsSpinBox->setRange(-11.9, 11.9);
//    setAllMotorSpeedsSpinBox->setSingleStep(0.1);
//    setAllMotorSpeedsButton = new QPushButton("Set");

//    QFormLayout *motorCommandsSubLayout = new QFormLayout;
//    motorCommandsSubLayout->addRow("Moving speed for all motors:", setAllMotorSpeedsSpinBox);
//    motorCommandsSubLayout->addRow(setAllMotorSpeedsButton);

    openMotorValueEditorButton = new QPushButton("Open motor value editor");
    setAllMotorTorquesOffButton = new QPushButton("Set all motor torques OFF");
    setAllMotorTorquesOffButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setAllMotorTorquesOffButton->setStyleSheet("QPushButton { color: white; background-color: #AA0000 }");


    QGridLayout *motorCommandsSubLayout = new QGridLayout;
    row = 0;
    col = 0;
    motorCommandsSubLayout->addWidget(openMotorValueEditorButton, row, col++);
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

    QGridLayout* buttonsSubLayout = new QGridLayout;
    row = 0;
    col = 0;
    buttonsSubLayout->addWidget(initRosNodeButton, row++, col);
    buttonsSubLayout->addWidget(addPoseButton, row++, col);
    buttonsSubLayout->addWidget(removePoseButton, row++, col);
    buttonsSubLayout->addWidget(setStartStateButton, row++, col);
    buttonsSubLayout->addWidget(setGoalStateButton, row++, col);
    buttonsSubLayout->addWidget(planMotionButton, row++, col);
    buttonsSubLayout->addWidget(executeMotionButton, row++, col);
    buttonsSubLayout->addWidget(addToQueueButton, row++, col);
    buttonsSubLayout->addWidget(removeFromQueueButton, row++, col);


    QList<RobotPoseStruct> availablePosesList;
    QList<RobotPoseStruct> queuedPosesList;

    RobotPoseStruct poseStruct;
    for (int i = 0; i < 5; ++i)
    {
        //poseStruct.name = "test" + QString::number(i, 'g', 3);
        //poseStruct.name = "test" + QString("%1").arg(i, 2, 'f', 4, QChar('0'));
        //
        std::ostringstream oss;
        //oss.precision(4);
        oss.width(3);
        oss.fill('0');
        oss.setf(std::ios::fixed, std::ios::floatfield);
        oss << i;
        poseStruct.name = "test" + QString::fromStdString(oss.str());

        //poseStruct.jointState.position.resize(NUM_OF_MOTORS);
        for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
            poseStruct.jointState.position[dxlId] =
                    static_cast<float>(rand()) / (static_cast<float>(RAND_MAX)) * (2.618 + 2.618) - 2.618;

        availablePosesList.push_back(poseStruct);
    }


    availablePosesCustomListWidget = new CustomListWidget(availablePosesList, "Poses available", 0, this);
    //new QListWidgetItem("listWidget1TestItem", customListWidget->listWidget);

    queuedPosesCustomListWidget = new CustomListWidget(queuedPosesList, "Poses queued", 1, this);
    //new QListWidgetItem("listWidget2TestItem", customListWidget2->listWidget);

    QWidget* motorFeedbackWidget = new QWidget(this);
    motorFeedbackWidget->setLayout(motorFeedbackSubLayout);

    QWidget* buttonsWidget = new QWidget(this);
    buttonsWidget->setLayout(buttonsSubLayout);

//    QWidget* inputsWidget =  new QWidget(this);
//    inputsWidget->setLayout(inputsSubLayout);
//    QFrame* hLine1 = new QFrame(this);
//    hLine1->setFrameStyle(QFrame::HLine);
//    QFrame* hLine2 = new QFrame(this);
//    hLine2->setFrameStyle(QFrame::HLine);

    QWidget* motorCommandsWidget =  new QWidget(this);
    motorCommandsWidget->setLayout(motorCommandsSubLayout);

    QWidget* fileIoWidget =  new QWidget(this);
    fileIoWidget->setLayout(fileIoSubLayout);

    outputLog = new OutputLog;
    outputLog->setReadOnly(true);


    row = 0;
    col = 0;
    QGridLayout* layout = new QGridLayout;
    layout->addWidget(motorFeedbackWidget, row, col++);
    layout->addWidget(availablePosesCustomListWidget, row, col++);
    layout->addWidget(buttonsWidget, row, col++);
    layout->addWidget(queuedPosesCustomListWidget, row++, col++);
    col = 0;
//    layout->addWidget(hLine1, row++, col, 1, -1);
//    layout->addWidget(inputsWidget, row++, col, 1, -1);
//    layout->addWidget(hLine2, row++, col, 1, -1);
    layout->addWidget(motorCommandsWidget, row, col++);
    layout->addWidget(fileIoWidget, row++, col--, 1, 3);
    layout->addWidget(outputLog, row++, col, 1, -1);


    QWidget* widget = new QWidget(this);
    widget->setLayout(layout);
    setCentralWidget(widget);
    setGeometry(0, 0, 800, 600);
}


void MainWindow::initRosNode()
{
    outputLog->appendTimestamped("ROS node initialised");
    rosWorker->init();
}


void MainWindow::addPose()
{
//    RobotPoseStruct poseStruct;
//    poseStruct.name = "newOne" + QString::number(rand() % 100);
//    for (int i = 0; i < NUM_OF_MOTORS; ++i)
//        poseStruct.jointState.position[i] = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/10.0));
//    availablePosesCustomListWidget->add(poseStruct);

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
    //robotFeedbackTimer->start(1000);
}


void MainWindow::nodeDisconnectedFromRosMaster()
{
    outputLog->appendTimestamped("ROS node disconnected from master");
    //robotFeedbackTimer->stop();
}


void MainWindow::enableMotionButtons()
{
    addPoseButton->setEnabled(true);
    //removePoseButton->setEnabled(true);
    setStartStateButton->setEnabled(true);
    setGoalStateButton->setEnabled(true);
    planMotionButton->setEnabled(true);
    executeMotionButton->setEnabled(true);

    openMotorValueEditorButton->setEnabled(true);
}


void MainWindow::disableMotionButtons()
{
    addPoseButton->setEnabled(false);
    //removePoseButton->setEnabled(false);
    setStartStateButton->setEnabled(false);
    setGoalStateButton->setEnabled(false);
    planMotionButton->setEnabled(false);
    executeMotionButton->setEnabled(false);

    openMotorValueEditorButton->setEnabled(false);
}


void MainWindow::updateJointStateValues(sensor_msgs::JointState js)
{
    // js[0] is empty/unused
    if ( (js.position.size() > NUM_OF_MOTORS) && (js.velocity.size() > NUM_OF_MOTORS) )
    {
        for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
        {
            // -2.618..2.618 rad, converted to -2700..2700 int range
            presentPosSliders[dxlId - 1]->setValue(js.position[dxlId] * 1000);

            std::ostringstream oss;
            oss.precision(4);
            oss.width(8);
            oss.fill(' ');
            oss.setf(std::ios::fixed, std::ios::floatfield);
            oss << js.position[dxlId];
            presentPosLineEdits[dxlId - 1]->setText(QString::fromStdString(oss.str()));

            oss.str("");
            oss.width(8);  // Not 'sticky'
            oss << js.velocity[dxlId];
            presentSpeedLineEdits[dxlId - 1]->setText(QString::fromStdString(oss.str()));
        }
    }
}


void MainWindow::updateSecondaryRobotValues(sensor_msgs::JointState js)
{
    // js[0] is empty/unused
    if ( (js.position.size() > NUM_OF_MOTORS) && (js.velocity.size() > NUM_OF_MOTORS) )
    {
        for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
        {
            std::ostringstream oss;
            oss.precision(4);
            oss.width(8);
            oss.fill(' ');
            oss.setf(std::ios::fixed, std::ios::floatfield);
            oss << js.position[dxlId];
            goalPosLineEdits[dxlId - 1]->setText(QString::fromStdString(oss.str()));

            oss.str("");
            oss.width(8);  // Not 'sticky'
            oss << js.velocity[dxlId];
            movingSpeedLineEdits[dxlId - 1]->setText(QString::fromStdString(oss.str()));
        }
    }
}


void MainWindow::updateJointStateValuesFromPose(const QModelIndex &modelIndex)
{
    // js[0] is empty/unused
    sensor_msgs::JointState js = availablePosesCustomListWidget->
            getRobotPosesListModel()->getCurrentPose(modelIndex).jointState;
    if (js.position.size() > NUM_OF_MOTORS)
    {
    for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
        presentPosSliders[dxlId - 1]->setSecondValue(js.position[dxlId] * 1000);
    }
}


void MainWindow::openMotorValueEditor()
{
    motorValueEditor->show();

    QRect screenGeometry = QApplication::desktop()->screenGeometry();
    motorValueEditor->move( screenGeometry.center() - motorValueEditor->rect().center() );
}


void MainWindow::quit()
{
    QApplication::quit();
}

