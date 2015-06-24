
#include "ros/ros.h"
#include "usb2ax_controller/AX12State.h"
#include "usb2ax_controller/AXS1State.h"
#include "usb2ax_controller/SendToAX.h"
#include <stdio.h>


void motorStateCallback(const usb2ax_controller::AX12State::ConstPtr& msg)
{
    ROS_INFO("-- Received motor message! --");
    ROS_INFO("Position: %d", msg->position);
    ROS_INFO("Speed: %d", msg->speed);
    ROS_INFO("Load: %d", msg->load);
    ROS_INFO("Voltage: %d", msg->voltage);
    ROS_INFO("Temperature: %d", msg->temperature);
    ROS_INFO("Moving: %d", msg->isMoving);
    ROS_INFO("-- End of motor message --");
}


void sensorStateCallback(const usb2ax_controller::AXS1State::ConstPtr& msg)
{
    ROS_INFO("-- Received sensor message! --");
    ROS_INFO("IR left: %d", msg->irLeft);
    ROS_INFO("IR centre: %d", msg->irCentre);
    ROS_INFO("IR right: %d", msg->irRight);
    ROS_INFO("Light left: %d", msg->lightLeft);
    ROS_INFO("Light centre: %d", msg->lightCentre);
    ROS_INFO("Light right: %d", msg->lightRight);
    ROS_INFO("Obstacle detected: %d", msg->irObstacleDetected);
    ROS_INFO("Light detected: %d", msg->isLightDetected);
    ROS_INFO("Sound: %d", msg->sound);
    ROS_INFO("Sound detected count: %d", msg->soundDetectedCount);
    ROS_INFO("Sound detected time: %d", msg->soundDetectedTime);
    ROS_INFO("-- End of sensor message --");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_listener");

    ros::NodeHandle n;

    ros::Subscriber motorSub = n.subscribe("State/Motor1", 1000, motorStateCallback);
    ros::Subscriber sensorSub = n.subscribe("State/Sensor100", 1000, sensorStateCallback);
    ros::spin();

//    ros::ServiceClient client = n.serviceClient<usb2ax_controller::SendToAX>("SendToAX");
//    while(1)
//    {
//        usb2ax_controller::SendToAX srv;
//        ROS_INFO("Enter ID: ");
//        scanf("%d", &srv.request.dxlID);
//        ROS_INFO("Enter address: ");
//        scanf("%d", &srv.request.address);
//        ROS_INFO("Enter value: ");
//        scanf("%d", &srv.request.value);
//        if (client.call(srv))
//        {
//          ROS_INFO("TX success: %d", srv.response.txSuccess);
//        }
//        else
//        {
//          ROS_ERROR("Failed to call service.");
//          return -1;
//        }
//    }

    return 0;
}

