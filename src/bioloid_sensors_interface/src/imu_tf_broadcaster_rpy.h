#ifndef IMU_TF_BROADCASTER_RPY_H
#define IMU_TF_BROADCASTER_RPY_H

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_broadcaster.h"

class Broadcaster
{
public:
    Broadcaster();
    virtual ~Broadcaster();
    void rpyCallback(const geometry_msgs::Vector3::ConstPtr& msg);

    tf::TransformBroadcaster* tfBroadcaster;
    float roll;
    float pitch;
    float yaw;
};

#endif // IMU_TF_BROADCASTER_RPY_H
