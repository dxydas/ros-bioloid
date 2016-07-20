#include "pidbalancerwidget.h"
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QStringList>
#include <qt5/QtWidgets/QVBoxLayout>
#include <qt5/QtWidgets/QHBoxLayout>
#include <qt5/QtWidgets/QAbstractButton>
#include <qt5/QtWidgets/QLabel>


PidWorker::PidWorker(RosWorker* rw, SimplePid* pid, QTextEdit* logTextEdit) :
    rw(rw), pid(pid), running(true), paused(true), logTextEdit(logTextEdit)
{
    // Initialise setpoint
    SP = 0.0;

    // Simple Moving Average
    SMA = 0.0;
    SMA_n = 10;
    SMA_window = std::deque<float>(SMA_n, 0);
}


void PidWorker::doWork()
{
    initialiseMotors();

    while (running)
    {
        if (!paused)
            stepPid();
        QThread::msleep(1);
    }

    emit finished();
}


void PidWorker::stepPid()
{
    if ( !rw->isConnectedToRosMaster() )
        return;

    try
    {
        rw->getListener()->lookupTransform("odom", "imu_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        QThread::msleep(1000);
        return;
    }

    // Moving average filter for pitch angle from IMU
    q = transform.getRotation();
    SMA_newValue = asin( -2.0*(q.x()*q.z() - q.y()*q.w()) / q.length() );
    SMA_oldValue = SMA_window.front();
    SMA += (SMA_newValue - SMA_oldValue)/SMA_n;
    SMA_window.pop_front();
    SMA_window.push_back(SMA_newValue);

    // PID control
    PV = SMA;
    output = pid->update(SP - PV, PV);

    // Trigger graph update
    emit ioGraphDataUpdated(SP, PV);

    if ( fabs(output) >= 0.0116 )
    {
        // Set motor speeds - ankle swing joints
        setMotorParamSrv.request.value = output;
        setMotorParamSrv.request.dxlID = 15;
        rw->setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv);
        setMotorParamSrv.request.dxlID = 16;
        rw->setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv);

        // Set motor outputs - ankle swing joints
        position = -PV;
        setMotorParamSrv.request.value = position;
        setMotorParamSrv.request.dxlID = 15;
        rw->setMotorGoalPositionInRadClient.call(setMotorParamSrv);
        setMotorParamSrv.request.dxlID = 16;
        rw->setMotorGoalPositionInRadClient.call(setMotorParamSrv);

        emit newLogMessageReady( "Output: " + QString::number(output) );
        emit newLogMessageReady( "Position: " + QString::number(position) );
    }
    else
        emit newLogMessageReady( "Speed too low: " + QString::number(output) );
}


void PidWorker::initialiseMotors()
{
    // Set slow speed
    setMotorParamSrv.request.dxlID = 254;
    setMotorParamSrv.request.value = 1.0;
    rw->setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv);
    QThread::msleep(500);

    // Home all motors
    rw->homeAllMotorsClient.call(emptySrv);
    // Personalised fix for left_hip_swing_joint which is slightly offset from home position
    setMotorParamSrv.request.dxlID = 12;
    setMotorParamSrv.request.value = 0.1173;
    rw->setMotorGoalPositionInRadClient.call(setMotorParamSrv);
    QThread::msleep(3000);

    // Set full speed
    setMotorParamSrv.request.dxlID = 254;
    setMotorParamSrv.request.value = 0.0;
    rw->setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv);
    QThread::msleep(500);
}


PidBalancerWidget::PidBalancerWidget(RosWorker* rosWorker, QWidget* parent) :
    mRosWorker(rosWorker), QWidget(parent)
{
    float maxSpeed = 12.276;
    pid = new SimplePid(0.0, 0.0, 0.0, maxSpeed, -maxSpeed);
    pidWidget = new PidWidget(pid, this);

    ioGraph = new SensorGraph(QStringList({"SP", "PV"}), "Input / Output", this);

    toggleBalancingButton = new QPushButton("Start Balancing");
    toggleBalancingButton->setCheckable(true);

    QVBoxLayout* ankleBalancingLayout = new QVBoxLayout;
    ankleBalancingLayout->addWidget(ioGraph);
    ankleBalancingLayout->addWidget(pidWidget, 0, Qt::AlignCenter);
    ankleBalancingLayout->addWidget(toggleBalancingButton);

    ankleBalancingGroupBox = new QGroupBox("Ankle Balancing");
    //ankleBalancingGroupBox->setMinimumSize(800, 600);
    ankleBalancingGroupBox->setLayout(ankleBalancingLayout);

    QLabel* logLabel = new QLabel("Log:");
    logTextEdit = new QTextEdit;
    logTextEdit->setReadOnly(true);
    QVBoxLayout* logLayout = new QVBoxLayout;
    logLayout->addWidget(logLabel);
    logLayout->addWidget(logTextEdit);
    logLayout->addStretch();

    QHBoxLayout* mainLayout = new QHBoxLayout(this);
    mainLayout->addWidget(ankleBalancingGroupBox);
    mainLayout->addLayout(logLayout);

    setLayout(mainLayout);

    ioGraph->setMinimumHeight(200);
    pidWidget->setMinimumWidth(500);
    logTextEdit->setMaximumSize(300, 100);
    setMaximumSize(1300, 700);

    elapsedTimer = new QElapsedTimer();

    workerThread = new QThread;
    pidWorker = new PidWorker(mRosWorker, pid, logTextEdit);
    pidWorker->moveToThread(workerThread);

    connect( toggleBalancingButton, SIGNAL(clicked(bool)), this, SLOT(setBalancingActive(bool)) );
    connect( pidWorker, SIGNAL(newLogMessageReady(QString)), this, SLOT(appendLogMessage(QString)) );
    connect( workerThread, SIGNAL(started()), pidWorker, SLOT(doWork()) );
    connect( pidWorker, SIGNAL(finished()), workerThread, SLOT(quit()) );
    connect( pidWorker, SIGNAL(finished()), pidWorker, SLOT(deleteLater()) );
    connect( workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()) );
    connect( pidWorker, SIGNAL(ioGraphDataUpdated(float, float)),
             this, SLOT(updateIoGraphData(float, float)) );

    workerThread->start();
    elapsedTimer->start();
}


PidBalancerWidget::~PidBalancerWidget()
{
    pidWorker->stop();
}


void PidBalancerWidget::setBalancingActive(bool activate)
{
    if (activate)
    {
        pidWorker->resume();
        toggleBalancingButton->setText("Stop Balancing");
    }
    else
    {
        pidWorker->pause();
        toggleBalancingButton->setText("Start Balancing");
    }
}


void PidBalancerWidget::updateIoGraphData(float SP, float PV)
{
    int t = elapsedTimer->elapsed();
    ioGraph->appendData(0, t, SP);
    ioGraph->appendData(1, t, PV);
}


void PidBalancerWidget::appendLogMessage(QString text)
{
    // Use a signal/slot here, because QtWidget and subclasses are not reentrant, so they can only be used from the
    // main thread
    logTextEdit->append(text);
}
