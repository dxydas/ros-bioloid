#include "moveit_api_test.h"
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "moveit_api_test");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // This sleep is ONLY to allow Rviz to come up
    sleep(10.0);

    bool success;


//    moveit::planning_interface::MoveGroup robotGroup("robot");
//    robotGroup.setNamedTarget("default_home");

//    moveit::planning_interface::MoveGroup::Plan my_plan1;
//    success = robotGroup.plan(my_plan1);
//    ROS_INFO("Visualizing move %s", success ? "SUCCESS" : "FAILED");


//    robotGroup.move();
//    sleep(2.0);


    moveit::planning_interface::MoveGroup group("right_leg");
    //group.setEndEffector("right_foot_ee");

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher =
            node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("End-effector link: %s", group.getEndEffectorLink().c_str());
    //ROS_INFO("End-effector: %s", group.getEndEffector().c_str());

    // Options:
    // - SBLkConfigDefault
    // - ESTkConfigDefault
    // - LBKPIECEkConfigDefault
    // - BKPIECEkConfigDefault
    // - KPIECEkConfigDefault
    // - RRTkConfigDefault
    // - RRTConnectkConfigDefault
    // - RRTstarkConfigDefault
    // - TRRTkConfigDefault
    // - PRMkConfigDefault
    // - PRMstarkConfigDefault
    //group.setPlannerId("PRMkConfigDefault");


    geometry_msgs::Pose start_pose = group.getCurrentPose().pose;
    geometry_msgs::Pose target_pose = group.getCurrentPose().pose;


    // Planning to a Pose goal
    target_pose = group.getCurrentPose().pose;
    //target_pose.position.x += 0.02;    // Foot forward
    //target_pose.position.y -= 0.02;    // Foot out
    target_pose.position.z += 0.02;      // Foot up
    group.setGoalTolerance(0.01);
    group.setPoseTarget(target_pose);
    //group.setJointValueTarget(target_pose);  // This also works!

    // Now, we call the planner to compute the plan
    // and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroup::Plan my_plan;
    success = group.plan(my_plan);
    ROS_INFO("Visualizing move %s", success ? "SUCCESS" : "FAILED");
    // Sleep to give Rviz time to visualize the plan
    sleep(10.0);


    // Cartesian Paths
    std::vector<geometry_msgs::Pose> waypoints;

//    target_pose.position.x += 0.05;
//    target_pose.position.z -= 0.02;
//    waypoints.push_back(target_pose);  // up and out
//
//    target_pose.position.y -= 0.02;
//    waypoints.push_back(target_pose);  // right
//
//    target_pose.position.z += 0.02;
//    target_pose.position.y += 0.02;
//    target_pose.position.x -= 0.02;
//    waypoints.push_back(target_pose);  // back to start

//    target_pose.orientation.w = sqrt(2.0);
//    target_pose.orientation.x = 0.0;
//    target_pose.orientation.y = 0.0;
//    target_pose.orientation.z = sqrt(2.0);
//    waypoints.push_back(target_pose);


//    // Ellipse
//    start_pose = group.getCurrentPose().pose;
//    target_pose = start_pose;
//    float ellipseA = 0.02;
//    float ellipseB = 0.02;
//    int N = 50;
//    for (int i = 0; i < N; ++i)
//    {
//        float t = i/(float)(N)*2*M_PI;
//        target_pose.position.x = ellipseA*cos(t) + start_pose.position.x;
//        target_pose.position.z = ellipseB*sin(t) + start_pose.position.z + ellipseB;
//        waypoints.push_back(target_pose);
//        ROS_INFO("Pose x: %g, z: %g", target_pose.position.x, target_pose.position.z);
//    }

//    //target_pose.position.x += 0.02;    // Foot forward
//    //target_pose.position.y -= 0.02;    // Foot out
//    target_pose.position.z += 0.02;      // Foot up
//    waypoints.push_back(target_pose);

//    group.setGoalTolerance(0.01);

//    moveit_msgs::RobotTrajectory trajectory;
//    double fraction;
//    fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, false);
//    ROS_INFO("Visualizing cartesian path (%.2f%% achieved)", fraction * 100.0);
//    // Sleep to give Rviz time to visualize the plan
//    sleep(10.0);


    ros::shutdown();

    return 0;
}
