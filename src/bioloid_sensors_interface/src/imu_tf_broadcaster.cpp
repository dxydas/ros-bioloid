
#include "imu_tf_broadcaster.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_tf_broadcaster");
    ros::NodeHandle n;
    ros::Rate loop_rate(1000);  // Hz

    Broadcaster broadcaster;

    ros::Subscriber accelSub   = n.subscribe("accel",   1000, &Broadcaster::accelCallback,   &broadcaster);
    ros::Subscriber magnetSub  = n.subscribe("magnet",  1000, &Broadcaster::magnetCallback,  &broadcaster);
    ros::Subscriber headingSub = n.subscribe("heading", 1000, &Broadcaster::headingCallback, &broadcaster);
    ros::Subscriber gyroSub    = n.subscribe("gyro",    1000, &Broadcaster::gyroCallback,    &broadcaster);

    while(n.ok())
    {
        //broadcaster.updateTimers();
        //broadcaster.updateRotation();
        broadcaster.tfBroadcaster->sendTransform(
            tf::StampedTransform(
                tf::Transform(broadcaster.getQ(), tf::Vector3(0.0, 0.0, 0.0)),
                ros::Time::now(), "odom", "imu_link") );
        ros::spinOnce();
        loop_rate.sleep();

//        std::cout << "dt: " << broadcaster.getDt() << std::endl;
//        std::cout << "q.x: " << broadcaster.getQ().x() << std::endl;
//        std::cout << "q.y: " << broadcaster.getQ().y() << std::endl;
//        std::cout << "q.z: " << broadcaster.getQ().z() << std::endl;
//        std::cout << "q.w: " << broadcaster.getQ().w() << std::endl;
//        std::cout << "----" << std::endl;
    }

    return 0;
}


Broadcaster::Broadcaster() :
    prevt(ros::Time::now().toSec()),
    dt(0.0),
    timeConst(1.0),
    filterCoeff(0.0)
{
    q.setRPY(0.0, 0.0, 0.0);
    tfBroadcaster = new tf::TransformBroadcaster();
}


Broadcaster::~Broadcaster()
{

}


void Broadcaster::accelCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    accel.setX(msg->x);
    accel.setY(msg->y);
    accel.setZ(msg->z);
}


void Broadcaster::magnetCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    magnet.setX(msg->x);
    magnet.setY(msg->y);
    magnet.setZ(msg->z);
}


void Broadcaster::headingCallback(const std_msgs::Float32::ConstPtr& msg)
{
    heading = msg->data;
}


void Broadcaster::gyroCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    dt = ros::Time::now().toSec() - prevt;

    gyro.setX(msg->x);
    gyro.setY(msg->y);
    gyro.setZ(msg->z);

    angularVel.setX(gyro.x());
    angularVel.setY(gyro.y());
    angularVel.setZ(gyro.z());

    filterCoeff = timeConst / (timeConst + dt);

    // Use accelerometer and magnetometer data to correct gyro drift
    correctOrientation();

    updateRotation();

    prevt = ros::Time::now().toSec();

//    std::cout << "angularVel x: " << angularVel.x() << std::endl;
//    std::cout << "angularVel y: " << angularVel.y() << std::endl;
//    std::cout << "angularVel z: " << angularVel.z() << std::endl;
//    std::cout << "----" << std::endl;
}


void Broadcaster::updateRotation()
{
    // New quaternion, from axis-angle notation for gyro
    tf::Quaternion qNew(angularVel.normalized(), angularVel.length()*dt);

    // Update previous value
    q *= qNew;
    q.normalize();
}


void Broadcaster::correctOrientation()
{
    // Use acceleration data only if vector is close to 1 g
    if ( fabs(accel.length() - 1) <= 0.1 )
    {
        // These vectors have to be perpendicular.
        // As there is no guarantee that North is perpendicular to Down,
        // set North to the cross product of East and Down.
        // Gravity is measured as an upward acceleration.
        tf::Vector3 Down(accel);  // Should be -accel, but not sure why this produces inverted axes!?
        tf::Vector3 E = Down.cross(magnet);
        tf::Vector3 N = E.cross(Down);

        Down.normalize();
        E.normalize();
        N.normalize();

        // The rows of the rotation matrix represent the coordinates in the original
        // space of unit vectors along the coordinate axes of the rotated space
        tf::Matrix3x3 gyroRotMat = tf::Matrix3x3(q);

        // Correction vector
        tf::Vector3 cv = ( N.cross(gyroRotMat.getRow(0)) +
                           E.cross(gyroRotMat.getRow(1)) +
                           Down.cross(gyroRotMat.getRow(2)) ) * filterCoeff;

        angularVel += cv;
    }
}

