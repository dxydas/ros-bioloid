//#include <curses.h>
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"
#include "usb2ax_controller/SetMotorParam.h"
#include "simplePID.h"
#include <deque>


int main(int argc, char **argv)
{
    //initscr();  // ncurses
    //timeout(0);

    ros::init(argc, argv, "test_balancer");

    ros::NodeHandle n;

    //ros::ServiceClient getMotorCurrentPositionInRadClient =
    //        n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorCurrentPositionInRad");
    ros::ServiceClient setMotorGoalPositionInRadClient =
            n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorGoalPositionInRad");
    ros::ServiceClient setMotorGoalSpeedInRadPerSecClient =
            n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorGoalSpeedInRadPerSec");
    ros::ServiceClient homeAllMotorsClient =
            n.serviceClient<std_srvs::Empty>("HomeAllMotors");

    //usb2ax_controller::GetMotorParam getMotorParamSrv;
    usb2ax_controller::SetMotorParam setMotorParamSrv;
    std_srvs::Empty emptySrv;

    // Set slow speed
    setMotorParamSrv.request.dxlID = 254;
    setMotorParamSrv.request.value = 1.0;
    setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv);
    ros::Duration(0.5).sleep();

    // Home all motors
    homeAllMotorsClient.call(emptySrv);
    // Temp fix for left_hip_swing_joint
    setMotorParamSrv.request.dxlID = 12;
    setMotorParamSrv.request.value = 0.1173;
    setMotorGoalPositionInRadClient.call(setMotorParamSrv);
    ros::Duration(3).sleep();

    // Set full speed
    setMotorParamSrv.request.dxlID = 254;
    setMotorParamSrv.request.value = 0.0;
    setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv);
    ros::Duration(0.5).sleep();

    tf::TransformListener listener;
    tf::StampedTransform transform;

    tf::Quaternion q;
    float position;

    float Ku, Tu, Kp, Ki, Kd;
    float SP, PV, output;
    float maxSpeed = 11.8668;

    SP = 0.0;

    // Ziegler–Nichols tuning
//    Ku = 0.5;
//    Tu = 0.7;

    // PI
//    Kp = 0.45*Ku;
//    Ki = 1.2*Kp/Tu;
//    Kd = 0.0;

    // PID
//    Kp = 0.6*Ku;
//    Ki = 2*Kp/Tu;
//    Kd = Kp*Tu/8.0;

    Kp = 2.0;
    Ki = 0.1;
    Kd = 0.0;

    std::cout.precision(6);
    std::cout.setf(std::ios::fixed, std::ios::floatfield);

    std::cout << "Ku: \t" << Ku << std::endl;
    std::cout << "Tu: \t" << Tu << std::endl;
    std::cout << "Kp: \t" << Kp << std::endl;
    std::cout << "Ki: \t" << Ki << std::endl;
    std::cout << "Kd: \t" << Kd << std::endl;

    SimplePID pid(Kp, Ki, Kd, -maxSpeed, maxSpeed);

    // Simple Moving Average
    float SMA = 0.0;
    int SMA_n = 10;
    std::deque<float> SMA_window;
    for (int i=0; i<SMA_n; ++i)
        SMA_window.push_back(0.0);
    float SMA_newValue, SMA_oldValue;

    while (n.ok())
    {
        //int i = getch();
        //printw("Entered: %d ", i);

        try
        {
            listener.lookupTransform("odom", "imu_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

        // Moving average filter for pitch angle from IMU
        q = transform.getRotation();
        SMA_newValue = asin( -2.0*(q.x()*q.z() - q.y()*q.w()) / q.length() );
        SMA_oldValue = SMA_window.front();
        SMA += (SMA_newValue - SMA_oldValue)/SMA_n;
        SMA_window.pop_front();
        SMA_window.push_back(SMA_newValue);

        // PID control
        PV = SMA;
        output = pid.update(SP - PV, PV);
        if (output > maxSpeed)
            output = maxSpeed;
        else if (output < -maxSpeed)
            output = -maxSpeed;

        if ( fabs(output) >= 0.0116 )
        {
            // Set motor speeds - ankle swing joints
            setMotorParamSrv.request.value = output;
            setMotorParamSrv.request.dxlID = 15;
            setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv);
            setMotorParamSrv.request.dxlID = 16;
            setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv);

            // Set motor outputs - ankle swing joints
            position = -PV;
            setMotorParamSrv.request.value = position;
            setMotorParamSrv.request.dxlID = 15;
            setMotorGoalPositionInRadClient.call(setMotorParamSrv);
            setMotorParamSrv.request.dxlID = 16;
            setMotorGoalPositionInRadClient.call(setMotorParamSrv);

            std::cout << "pitch angle: " << PV;
            std::cout << "\t output speed: " << output;
            std::cout << "\t output pos: " << position << std::endl;
        }
        else
            std::cout << "Speed too low: " << output << std::endl;

        ros::Duration(0.1).sleep();
    }
    //endwin();  // ncurses

    return 0;
}

