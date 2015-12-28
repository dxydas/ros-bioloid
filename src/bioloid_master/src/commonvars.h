#ifndef COMMONVARS_H
#define COMMONVARS_H

#include <qt5/QtCore/QString>
#include "sensor_msgs/JointState.h"
#include "../usb2ax_controller/src/ax12ControlTableMacros.h"

#define NUM_OF_MOTORS 18

struct RobotPoseStruct
{
    QString name;
    sensor_msgs::JointState jointState;
    RobotPoseStruct()
    {
        name = "";
        jointState.position.resize(NUM_OF_MOTORS);
        jointState.velocity.resize(NUM_OF_MOTORS);
        jointState.effort.resize(NUM_OF_MOTORS);
    }
};

#endif // COMMONVARS_H
