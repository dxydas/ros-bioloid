#ifndef MOVEITHANDLER_H
#define MOVEITHANDLER_H

#include <qt5/QtCore/QList>
#include <qt5/QtWidgets/QWidget>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include "commonvars.h"

class MoveItHandler : public QWidget
{
    Q_OBJECT
public:
    explicit MoveItHandler(QWidget* parent = 0);
    void init();

signals:

public slots:
    void setCurrentAsStartState();
    void setCurrentAsGoalState();
    void planMotion();
    void executeMotion();
    void planAndExecuteChain(QList<RobotPoseStruct> robotPosesList);

private:
    moveit::planning_interface::MoveGroup* group;
    moveit::planning_interface::MoveGroup::Plan plan;
//    geometry_msgs::Pose startPose;
//    geometry_msgs::Pose targetPose;
    std::vector<double> startJointValues;
    std::vector<double> targetJointValues;
};

#endif // MOVEITHANDLER_H
