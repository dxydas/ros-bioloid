#ifndef IMU_TF_BROADCASTER_H
#define IMU_TF_BROADCASTER_H

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include "tf/transform_broadcaster.h"

class Broadcaster
{
public:
    Broadcaster();
    virtual ~Broadcaster();
    void accelCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void magnetCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void headingCallback(const std_msgs::Float32::ConstPtr& msg);
    void gyroCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    //void updateTimers();
    void updateRotation();
    void correctOrientation();
    tf::TransformBroadcaster* tfBroadcaster;
    tf::Quaternion getQ() const {return q;}
    double getPrevt() const {return prevt;}
    double getDt() const {return dt;}
    float getTimeConst() const {return timeConst;}
    void setTimeConst(float value) {timeConst = value;}

private:
    tf::Vector3 accel;
    tf::Vector3 magnet;
    float heading;
    tf::Vector3 gyro;

    tf::Vector3 angularVel;
    tf::Quaternion q;
    double prevt;
    double dt;
    float timeConst;
    float filterCoeff;
};

#endif // IMU_TF_BROADCASTER_H
