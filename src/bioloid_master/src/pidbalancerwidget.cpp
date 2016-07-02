#include "pidbalancerwidget.h"
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QStringList>
#include <qt5/QtWidgets/QVBoxLayout>
#include <qt5/QtWidgets/QAbstractButton>


PidWorker::PidWorker(RosWorker* rw, SimplePid* pid, QMutex* mutex) :
    rw(rw), pid(pid), locker(mutex), running(true), paused(true)
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
    // Initialise motors

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

    while (running)
    {
        if (paused)
            pauseCondition.wait(locker.mutex());

        stepPid();
    }

    emit finished();
}


void PidWorker::pause()
{
    paused = true;
}


void PidWorker::resume()
{
    paused = false;
    pauseCondition.wakeAll();
}


void PidWorker::stop()
{
    running = false;
}


void PidWorker::stepPid()
{
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

    // Graph updates
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

        //            std::cout << "pitch angle: " << PV;
        //            std::cout << "\t output speed: " << output;
        //            std::cout << "\t output pos: " << position << std::endl;
    }
    //        else
    //            std::cout << "Speed too low: " << output << std::endl;

    QThread::msleep(100);
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

    ankleBalancingGroupBox = new QGroupBox("Ankle Balancing", this);
    ankleBalancingGroupBox->setMinimumSize(800, 600);
    ankleBalancingGroupBox->setLayout(ankleBalancingLayout);

    ioGraph->setMinimumHeight(200);
    pidWidget->setMinimumWidth(500);
    setMinimumSize(1300, 700);

    elapsedTimer = new QElapsedTimer();

    workerThread = new QThread;
    pidWorker = new PidWorker(mRosWorker, pid, &moveMutex);
    pidWorker->moveToThread(workerThread);
    connect( workerThread, SIGNAL(started()), pidWorker, SLOT(doWork()) );
    connect( pidWorker, SIGNAL(finished()), workerThread, SLOT(quit()) );
    connect( pidWorker, SIGNAL(finished()), pidWorker, SLOT(deleteLater()) );
    connect( workerThread, SIGNAL(finished()), workerThread, SLOT(deleteLater()) );
    connect( pidWorker, SIGNAL(ioGraphDataUpdated(float, float)),
             this, SLOT(updateIoGraphData(float, float)) );
    connect( toggleBalancingButton, SIGNAL(clicked(bool)), this, SLOT(setBalancingActive(bool)) );

    workerThread->start();
    elapsedTimer->start();
}


PidBalancerWidget::~PidBalancerWidget()
{
    pidWorker->stop();
    workerThread->quit();
    workerThread->wait();
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
