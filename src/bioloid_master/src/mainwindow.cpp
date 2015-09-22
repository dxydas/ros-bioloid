#include "mainwindow.h"
#include <iostream>
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

#define NUM_OF_MOTORS 18

Q_DECLARE_METATYPE(sensor_msgs::JointState)


MainWindow::MainWindow(int argc, char* argv[], QWidget* parent) :
    QMainWindow(parent)
{
    setUpLayout();
    outputLog->appendTimestamped("Application started");

    rosWorker = new RosWorker(argc, argv, "robot_gui");

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

    widget = new QWidget(this);
    QGridLayout* layout = new QGridLayout;
    outputLog = new OutputLog;

    outputLog->setReadOnly(true);

    widget->setLayout(layout);

    setCentralWidget(widget);
    setGeometry(0, 0, 800, 600);


    int row = 0;
    int col = 0;

    QLabel* motorLabel = new QLabel("Motor");
    QLabel* presentPositionLabel = new QLabel("Present position");
    QLabel* goalPositionLabel = new QLabel("Goal position");
    QLabel* presentSpeedLabel = new QLabel("Present speed");
    QLabel* movingSpeedLabel = new QLabel("Moving speed");
    motorLabel->setAlignment(Qt::AlignCenter);
    presentPositionLabel->setAlignment(Qt::AlignCenter);
    goalPositionLabel->setAlignment(Qt::AlignCenter);
    presentSpeedLabel->setAlignment(Qt::AlignCenter);
    movingSpeedLabel->setAlignment(Qt::AlignCenter);
    QGridLayout* motorFeedbackSubLayout = new QGridLayout;
    motorFeedbackSubLayout->addWidget(motorLabel, row, col++);
    motorFeedbackSubLayout->addWidget(presentPositionLabel, row, col++, 1, 2);
    col++;
    motorFeedbackSubLayout->addWidget(goalPositionLabel, row, col++);
    motorFeedbackSubLayout->addWidget(presentSpeedLabel, row, col++);
    motorFeedbackSubLayout->addWidget(movingSpeedLabel, row, col++);
    QVector<QLabel*> motorLineLabels(NUM_OF_MOTORS);
    presentPosSliders.resize(NUM_OF_MOTORS);
    presentPosLineEdits.resize(NUM_OF_MOTORS);
    goalPosLineEdits.resize(NUM_OF_MOTORS);
    presentSpeedLineEdits.resize(NUM_OF_MOTORS);
    movingSpeedLineEdits.resize(NUM_OF_MOTORS);
    for (int i = 0; i < NUM_OF_MOTORS; ++i)
    {
        motorLineLabels[i] = new QLabel(QString::number(i+1));
        presentPosSliders[i] = new DoubleSlider(Qt::Horizontal, this);
        presentPosSliders[i]->setMinimum(-2700);
        presentPosSliders[i]->setMaximum(2700);
        presentPosSliders[i]->setValue(0);
        presentPosSliders[i]->setSecondValue(0);
        //presentPosSliders[i]->setTickPosition(QSlider::TicksBothSides);
        //presentPosSliders[i]->setTickInterval(200);

        std::ostringstream oss;
        oss.precision(4);
        oss.width(7);
        oss.fill('0');
        oss.setf(std::ios::fixed, std::ios::floatfield);
        oss << 0.0;
        QString qStr = QString::fromStdString(oss.str());

        presentPosLineEdits[i] = new QLineEdit(qStr);
        goalPosLineEdits[i] = new QLineEdit(qStr);
        presentSpeedLineEdits[i] = new QLineEdit(qStr);
        movingSpeedLineEdits[i] = new QLineEdit(qStr);
        presentPosLineEdits[i]->setMaximumWidth(100);
        presentPosLineEdits[i]->setMaximumHeight(16);
        goalPosLineEdits[i]->setMaximumWidth(100);
        goalPosLineEdits[i]->setMaximumHeight(16);
        presentSpeedLineEdits[i]->setMaximumWidth(100);
        presentSpeedLineEdits[i]->setMaximumHeight(16);
        movingSpeedLineEdits[i]->setMaximumWidth(100);
        movingSpeedLineEdits[i]->setMaximumHeight(16);
        row = i + 1;
        col = 0;
        motorLineLabels[i]->setAlignment(Qt::AlignCenter);
        presentPosLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        goalPosLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        presentSpeedLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        movingSpeedLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        motorFeedbackSubLayout->addWidget(motorLineLabels[i], row, col++);
        motorFeedbackSubLayout->addWidget(presentPosSliders[i], row, col++);
        motorFeedbackSubLayout->addWidget(presentPosLineEdits[i], row, col++);
        motorFeedbackSubLayout->addWidget(goalPosLineEdits[i], row, col++);
        motorFeedbackSubLayout->addWidget(presentSpeedLineEdits[i], row, col++);
        motorFeedbackSubLayout->addWidget(movingSpeedLineEdits[i], row, col++);
    }
    motorFeedbackSubLayout->setColumnStretch(0, 0);
    motorFeedbackSubLayout->setColumnStretch(1, 2);
    motorFeedbackSubLayout->setColumnStretch(2, 1);
    motorFeedbackSubLayout->setColumnStretch(3, 1);
    motorFeedbackSubLayout->setColumnStretch(4, 1);


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


//    QFormLayout* inputsSubLayout = new QFormLayout;
//    QLineEdit* savedPosesFilePathLineEdit = new QLineEdit(defaultSavedPosesFilePath);
//    inputsSubLayout->addRow(new QLabel("Saved poses file path:"), savedPosesFilePathLineEdit);
//    inputsSubLayout->addRow(new QLabel("Placeholder:"), new QLineEdit("<placeholder>"));

    saveAvailablePosesFileButton = new QPushButton("Save available poses file");
    loadAvailablePosesFileButton = new QPushButton("Load available poses file");
    saveQueuedPosesFileButton = new QPushButton("Save queued poses file");
    loadQueuedPosesFileButton = new QPushButton("Load queued poses file");
    QGridLayout* ioSubLayout = new QGridLayout;
    row = 0;
    col = 0;
    ioSubLayout->addWidget(saveAvailablePosesFileButton, row, col++);
    ioSubLayout->addWidget(loadAvailablePosesFileButton, row++, col--);
    ioSubLayout->addWidget(saveQueuedPosesFileButton, row, col++);
    ioSubLayout->addWidget(loadQueuedPosesFileButton, row, col--);


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
        for (int i = 0; i < NUM_OF_MOTORS; ++i)
            poseStruct.jointState.position[i] = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10.0));

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

    QWidget* ioWidget =  new QWidget(this);
    ioWidget->setLayout(ioSubLayout);


    row = 0;
    col = 0;
    layout->addWidget(motorFeedbackWidget, row, col++);
    layout->addWidget(availablePosesCustomListWidget, row, col++);
    layout->addWidget(buttonsWidget, row, col++);
    layout->addWidget(queuedPosesCustomListWidget, row++, col++);
    col = 0;
//    layout->addWidget(hLine1, row++, col, 1, -1);
//    layout->addWidget(inputsWidget, row++, col, 1, -1);
//    layout->addWidget(hLine2, row++, col, 1, -1);
    layout->addWidget(ioWidget, row++, col, 1, -1);
    layout->addWidget(outputLog, row++, col, 1, -1);
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
//        poseStruct.jointState.position[i] = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10.0));
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
}


void MainWindow::disableMotionButtons()
{
    addPoseButton->setEnabled(false);
    //removePoseButton->setEnabled(false);
    setStartStateButton->setEnabled(false);
    setGoalStateButton->setEnabled(false);
    planMotionButton->setEnabled(false);
    executeMotionButton->setEnabled(false);
}


void MainWindow::updateJointStateValues(sensor_msgs::JointState js)
{
    // js[0] is empty/unused
    if ( (js.position.size() >= NUM_OF_MOTORS) && (js.velocity.size() >= NUM_OF_MOTORS) )
    {
        for (int dxlID = 1; dxlID <= NUM_OF_MOTORS; ++dxlID)
        {
            // -2.618..2.618 rad, converted to -2700..2700 int range
            presentPosSliders[dxlID - 1]->setValue(js.position[dxlID] * 1000);

            std::ostringstream oss;
            oss.precision(4);
            oss.width(7);
            oss.fill('0');
            oss.setf(std::ios::fixed, std::ios::floatfield);
            oss << js.position[dxlID];
            presentPosLineEdits[dxlID - 1]->setText(QString::fromStdString(oss.str()));

            oss.str("");
            oss.width(7);  // Not 'sticky'
            oss << js.velocity[dxlID];
            presentSpeedLineEdits[dxlID - 1]->setText(QString::fromStdString(oss.str()));
        }
    }
}


void MainWindow::updateSecondaryRobotValues(sensor_msgs::JointState js)
{
    // js[0] is empty/unused
    if ( (js.position.size() >= NUM_OF_MOTORS) && (js.velocity.size() >= NUM_OF_MOTORS) )
    {
        for (int dxlID = 1; dxlID <= NUM_OF_MOTORS; ++dxlID)
        {
            std::ostringstream oss;
            oss.precision(4);
            oss.width(7);
            oss.fill('0');
            oss.setf(std::ios::fixed, std::ios::floatfield);
            oss << js.position[dxlID];
            goalPosLineEdits[dxlID - 1]->setText(QString::fromStdString(oss.str()));

            oss.str("");
            oss.width(7);  // Not 'sticky'
            oss << js.velocity[dxlID];
            movingSpeedLineEdits[dxlID - 1]->setText(QString::fromStdString(oss.str()));
        }
    }
}


void MainWindow::updateJointStateValuesFromPose(const QModelIndex &modelIndex)
{
    // js[0] is empty/unused
    sensor_msgs::JointState js = availablePosesCustomListWidget->
            getRobotPosesListModel()->getCurrentPose(modelIndex).jointState;
    if (js.position.size() >= NUM_OF_MOTORS)
    {
    for (int dxlID = 1; dxlID <= NUM_OF_MOTORS; ++dxlID)
        presentPosSliders[dxlID - 1]->setSecondValue(js.position[dxlID] * 1000);
    }
}


void MainWindow::quit()
{
    QApplication::quit();
}

