#include "sensorgrapher.h"
#include <qt5/QtWidgets/QGridLayout>


SensorGrapher::SensorGrapher(RosWorker* rosWorker, QWidget* parent) :
    mRosWorker(rosWorker), QWidget(parent), numOfGraphs(5)
{
    setWindowTitle("Sensor Grapher");

    int row = 0;
    int col = 0;
    QGridLayout* gridLayout = new QGridLayout;
    sensorGraphs.resize(numOfGraphs);
    for (int i = 0; i < numOfGraphs; ++i)
    {
        QStringList lineNames;
        QString yLabel;
        switch (i)
        {
        case 0:
            lineNames << "X" << "Y" << "Z";
            yLabel = "Acceleration";
            break;
        case 1:
            lineNames << "X" << "Y" << "Z";
            yLabel = "Magnet";
            break;
        case 2:
            lineNames << "Heading";
            yLabel = "Heading";
            break;
        case 3:
            lineNames << "X" << "Y" << "Z";
            yLabel = "Gyro";
            break;
        case 4:
            lineNames << "1" << "2" << "3" << "4" << "5" << "6";
            yLabel = "Pressure";
            break;
        }
        sensorGraphs[i] = new SensorGraph(lineNames, yLabel, this);
        gridLayout->addWidget(sensorGraphs[i], row++, col);
    }
    setLayout(gridLayout);

    setMinimumSize(800, 600);

    elapsedTimer = new QElapsedTimer();

    connect( mRosWorker, SIGNAL(accelDataUpdated(geometry_msgs::Vector3)),
                                this, SLOT(updateAccelGraphData(geometry_msgs::Vector3)) );
    connect( mRosWorker, SIGNAL(magnetDataUpdated(geometry_msgs::Vector3)),
                                this, SLOT(updateMagnetGraphData(geometry_msgs::Vector3)) );
    connect( mRosWorker, SIGNAL(headingDataUpdated(std_msgs::Float32)),
                                this, SLOT(updateHeadingGraphData(std_msgs::Float32)) );
    connect( mRosWorker, SIGNAL(gyroDataUpdated(geometry_msgs::Vector3)),
                                this, SLOT(updateGyroGraphData(geometry_msgs::Vector3)) );
    connect( mRosWorker, SIGNAL(fsrsDataUpdated(std_msgs::Int16MultiArray)),
                                this, SLOT(updateFsrsGraphData(std_msgs::Int16MultiArray)) );

    elapsedTimer->start();
}


void SensorGrapher::updateAccelGraphData(geometry_msgs::Vector3 vec)
{
    int t = elapsedTimer->elapsed();
    sensorGraphs[0]->appendData(0, t, vec.x);
    sensorGraphs[0]->appendData(1, t, vec.y);
    sensorGraphs[0]->appendData(2, t, vec.z);
}


void SensorGrapher::updateMagnetGraphData(geometry_msgs::Vector3 vec)
{
    int t = elapsedTimer->elapsed();
    sensorGraphs[1]->appendData(0, t, vec.x);
    sensorGraphs[1]->appendData(1, t, vec.y);
    sensorGraphs[1]->appendData(2, t, vec.z);
}


void SensorGrapher::updateHeadingGraphData(std_msgs::Float32 val)
{
    int t = elapsedTimer->elapsed();
    sensorGraphs[2]->appendData(0, t, val.data);
}


void SensorGrapher::updateGyroGraphData(geometry_msgs::Vector3 vec)
{
    int t = elapsedTimer->elapsed();
    sensorGraphs[3]->appendData(0, t, vec.x);
    sensorGraphs[3]->appendData(1, t, vec.y);
    sensorGraphs[3]->appendData(2, t, vec.z);
}


void SensorGrapher::updateFsrsGraphData(std_msgs::Int16MultiArray arr)
{
    int t = elapsedTimer->elapsed();
    for (int i = 0; i < arr.data.size(); ++i)
        sensorGraphs[4]->appendData(i, t, arr.data[i]);
}
