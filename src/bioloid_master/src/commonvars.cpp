#include "commonvars.h"

RobotPose::RobotPose()
{
    name = "RobotPose";
    jointState.position.resize(NUM_OF_MOTORS, 0.0);
    jointState.velocity.resize(NUM_OF_MOTORS, 0.0);
    jointState.effort.resize(NUM_OF_MOTORS, 0.0);
    dwellTimeInSec = 0.0;
}
