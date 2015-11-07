#include "moveithandler.h"


MoveItHandler::MoveItHandler(QWidget* parent) :
    QWidget(parent)
{

}


void MoveItHandler::init()
{
    group = new moveit::planning_interface::MoveGroup("robot");
    group->setPlannerId("PRMkConfigDefault");
    group->setGoalTolerance(0.01);

    startPose = group->getCurrentPose().pose;
    targetPose = group->getCurrentPose().pose;
}


void MoveItHandler::setStartState()
{
    startPose = group->getCurrentPose().pose;
}


void MoveItHandler::setGoalState()
{
    targetPose = group->getCurrentPose().pose;
}


void MoveItHandler::planMotion()
{
    group->setPoseTarget(targetPose);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group to actually move the robot.
    bool success = group->plan(plan);
    ROS_INFO("Visualizing move %s", success ? "SUCCESS" : "FAILED");
}


void MoveItHandler::executeMotion()
{
    group->asyncExecute(plan);
}
