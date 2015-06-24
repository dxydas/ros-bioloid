
#include "imu_tf_broadcaster_rpy.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_tf_broadcaster");
    ros::NodeHandle n;
    ros::Rate loop_rate(1000);  // Hz

    Broadcaster broadcaster;

    ros::Subscriber rpySub = n.subscribe("rpy", 1000, &Broadcaster::rpyCallback, &broadcaster);

    tf::Quaternion q;

    while(n.ok())
    {
        q.setRPY(broadcaster.roll, broadcaster.pitch, broadcaster.yaw);

        broadcaster.tfBroadcaster->sendTransform(
            tf::StampedTransform(
                tf::Transform(q, tf::Vector3(0.0, 0.0, 0.0)),
                ros::Time::now(),"odom", "imu_link"));

        //ROS_INFO("Roll: %f", broadcaster.roll*180.0/M_PI);
        //ROS_INFO("Pitch: %f", broadcaster.pitch*180.0/M_PI);
        //ROS_INFO("Yaw: %f", broadcaster.yaw*180.0/M_PI);
        //ROS_INFO("----");

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}


Broadcaster::Broadcaster() :
    roll(0.0),
    pitch(0.0),
    yaw(0.0)
{
    tfBroadcaster = new tf::TransformBroadcaster();
}


Broadcaster::~Broadcaster()
{

}


void Broadcaster::rpyCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    roll = msg->x;
    pitch = msg->y;
    yaw = msg->z;
}

