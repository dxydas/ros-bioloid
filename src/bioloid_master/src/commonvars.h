#ifndef COMMONVARS_H
#define COMMONVARS_H

#include <qt5/QtCore/QString>
#include "sensor_msgs/JointState.h"

#define NUM_OF_MOTORS 18

class RobotPose
{
public:
    explicit RobotPose();
    QString name;
    sensor_msgs::JointState jointState;
    float dwellTimeInSec;
};

#endif // COMMONVARS_H
