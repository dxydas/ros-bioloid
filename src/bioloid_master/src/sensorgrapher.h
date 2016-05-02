#ifndef SENSORGRAPHER_H
#define SENSORGRAPHER_H

#include <qt5/QtCore/QVector>
#include <qt5/QtCore/QElapsedTimer>
#include <qt5/QtWidgets/QWidget>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include "rosworker.h"
#include "sensorgraph.h"

class SensorGrapher : public QWidget
{
    Q_OBJECT

public:
    explicit SensorGrapher(RosWorker* rosWorker, QWidget* parent = 0);

signals:

public slots:
    void updateAccelGraphData(geometry_msgs::Vector3 vec);
    void updateMagnetGraphData(geometry_msgs::Vector3 vec);
    void updateHeadingGraphData(std_msgs::Float32 val);
    void updateGyroGraphData(geometry_msgs::Vector3 vec);
    void updateFsrsGraphData(std_msgs::Int16MultiArray arr);

private:
    RosWorker* mRosWorker;
    int numOfGraphs;
    QVector<SensorGraph*> sensorGraphs;
    QElapsedTimer* elapsedTimer;
};

#endif // SENSORGRAPHER_H
