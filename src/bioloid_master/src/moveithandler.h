#ifndef MOVEITHANDLER_H
#define MOVEITHANDLER_H

#include <qt5/QtWidgets/QWidget>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>

class MoveItHandler : public QWidget
{
    Q_OBJECT
public:
    explicit MoveItHandler(QWidget* parent = 0);
    void init();

signals:

public slots:
    void setStartState();
    void setGoalState();
    void planMotion();
    void executeMotion();

private:
    moveit::planning_interface::MoveGroup* group;
    moveit::planning_interface::MoveGroup::Plan plan;
    geometry_msgs::Pose startPose;
    geometry_msgs::Pose targetPose;
};

#endif // MOVEITHANDLER_H
