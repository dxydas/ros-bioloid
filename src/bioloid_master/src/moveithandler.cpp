#include "moveithandler.h"
#include <qt5/QtCore/QVector>


MoveItHandler::MoveItHandler(OutputLog* outputLog, QWidget* parent) :
    outputLog(outputLog), QWidget(parent)
{
}


void MoveItHandler::initialise()
{
    group = new moveit::planning_interface::MoveGroup("robot");
    group->setPlannerId("PRMkConfigDefault");
    group->setGoalTolerance(0.01);

//    startPose = group->getCurrentPose().pose;
//    targetPose = group->getCurrentPose().pose;

    group->getCurrentState()->copyJointGroupPositions(
                group->getCurrentState()->getRobotModel()->getJointModelGroup(group->getName()),
                startJointValues );
    targetJointValues = startJointValues;

    emit initialised();
    outputLog->appendTimestamped("MoveIt! handler initialised");
}


void MoveItHandler::setCurrentAsStartState()
{
//    startPose = group->getCurrentPose().pose;
//    group->setStartState(startPose);
    group->getCurrentState()->copyJointGroupPositions(
                group->getCurrentState()->getRobotModel()->getJointModelGroup(group->getName()),
                startJointValues );
    group->setStartStateToCurrentState();
}


void MoveItHandler::setCurrentAsGoalState()
{
//    targetPose = group->getCurrentPose().pose;
//    group->setPoseTarget(targetPose);
    group->getCurrentState()->copyJointGroupPositions(
                group->getCurrentState()->getRobotModel()->getJointModelGroup(group->getName()),
                targetJointValues );
    group->setJointValueTarget(targetJointValues);
}


void MoveItHandler::planMotion()
{
    // Call the planner to compute the plan and visualize it.
    // Note that this is just planning, not asking move_group to actually move the robot.
    bool success = group->plan(plan);
    ROS_INFO("Visualising plan %s", success ? "SUCCESS" : "FAILED");
}


void MoveItHandler::executeMotion()
{
    group->asyncExecute(plan);
}


void MoveItHandler::planAndExecuteChain(QList<RobotPose> robotPosesList)
{
    setCurrentAsStartState();
    for (int i = 0; i < robotPosesList.size(); ++i)
    {
        targetJointValues = robotPosesList[i].jointState.position;
        group->setJointValueTarget(targetJointValues);
        planMotion();
        executeMotion();
    }
}
