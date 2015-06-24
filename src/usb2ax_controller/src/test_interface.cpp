#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "usb2ax_controller/SendToAX.h"
#include "usb2ax_controller/SetMotorParam.h"
#include "ax12ControlTableMacros.h"


void testArmWave(ros::ServiceClient setMotorGoalPositionInRadClient)
{
    usb2ax_controller::SetMotorParam srv;

    // Right arm: IDs 3, 5
    // Left arm:  IDs 4, 6

    float p3 = M_PI/2.0;
    float p5 = 0.0;
    float p4 = M_PI/2.0;
    float p6 = 0.0;

    float A = M_PI/4.0;
    float runTime = 2.0;
    float delayTime = runTime/2.0;
    float f1 = 1.0/runTime;
    float f2 = 2.0/runTime;
    float fi = 0.0;//M_PI/4.0;
    float samplingInterval = 0.1;
    int samples = (int)ceil(runTime/samplingInterval);
    int i2 = (int)ceil(delayTime/samplingInterval);

    // Initial position - Arms outstreched
    srv.request.dxlID = 3;
    srv.request.value = p3;
    setMotorGoalPositionInRadClient.call(srv);
    //
    srv.request.dxlID = 5;
    srv.request.value = p5;
    setMotorGoalPositionInRadClient.call(srv);
    //
    srv.request.dxlID = 4;
    srv.request.value = p4 + fi;
    setMotorGoalPositionInRadClient.call(srv);
    //
    srv.request.dxlID = 6;
    srv.request.value = p6 + fi;
    setMotorGoalPositionInRadClient.call(srv);
    //
    ros::Duration(2).sleep();

    // Wave
    ROS_DEBUG("Amplitude: %g", A);
    ROS_DEBUG("Runtime: %g", runTime);
    ROS_DEBUG("Frequency 1: %g", f1);
    ROS_DEBUG("Frequency 2: %g", f2);
    ROS_DEBUG("Arm 2 delay: %g", delayTime);
    ROS_DEBUG("Phase diff.: %g", fi);
    ROS_DEBUG("Sampling interval: %g", samplingInterval);
    ROS_DEBUG("Samples: %d", samples);
    ROS_DEBUG("i2: %d", i2);
    ROS_DEBUG("y1\t y2\t y3\t y4");
    float y1, y2, y3, y4;
    for (int i = 0; i < samples; ++i)
    {
        //ROS_DEBUG("i: %d", i);

        float t = i*samplingInterval;

        // Right arm
        y1 = A*sin( 2.0*M_PI*f1*t );
        y2 = A*sin( 2.0*M_PI*f2*t );
        //
        srv.request.dxlID = 3;
        srv.request.value = p3 + y1;
        setMotorGoalPositionInRadClient.call(srv);
        //
        srv.request.dxlID = 5;
        srv.request.value = p5 + y2;
        setMotorGoalPositionInRadClient.call(srv);

        if ( i > i2 )
        {
            // Left arm
            y3 = A*sin( 2.0*M_PI*f1*(t-delayTime) + fi );
            y4 = A*sin( 2.0*M_PI*f2*(t-delayTime) + fi );
            srv.request.dxlID = 4;
            srv.request.value = p4 + y3;
            setMotorGoalPositionInRadClient.call(srv);
            srv.request.dxlID = 6;
            srv.request.value = p6 + y4;
            setMotorGoalPositionInRadClient.call(srv);
            ROS_DEBUG("%g\t %g\t %g\t %g", y1, y2, y3, y4);
        }
        else
            ROS_DEBUG("%g\t %g\t", y1, y2);

        ros::Duration(samplingInterval).sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_interface");

    ros::NodeHandle n;

    // Service clients
    ros::ServiceClient sendToAXClient =
            n.serviceClient<usb2ax_controller::SendToAX>("SendToAX");
    ros::ServiceClient setMotorGoalPositionInRadClient =
            n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorGoalPositionInRad");
    ros::ServiceClient setMotorGoalSpeedInRadPerSecClient =
            n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorGoalSpeedInRadPerSec");
    ros::ServiceClient setMotorMaxTorqueInDecimalClient =
            n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorMaxTorqueInDecimal");
    ros::ServiceClient homeAllMotorsClient =
            n.serviceClient<std_srvs::Empty>("HomeAllMotors");

    usb2ax_controller::SendToAX sendToAXSrv;
    usb2ax_controller::SetMotorParam setMotorParamSrv;
    std_srvs::Empty emptySrv;

//    // Set max torque
//    SetMotorParamSrv.request.dxlID = 254;
//    SetMotorParamSrv.request.value = 0.8;
//    if (setMotorMaxTorqueInDecimalClient.call(SetMotorParamSrv))
//        ROS_INFO("TX success: %d", SetMotorParamSrv.response.txSuccess);
//    else
//    {
//        ROS_ERROR("Failed to call service.");
//        return -1;
//    }
//    ros::Duration(0.5).sleep();

    // Set slow speed
    setMotorParamSrv.request.dxlID = 254;
    setMotorParamSrv.request.value = 1.0;
    if (setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv))
        ROS_INFO("TX success: %d", setMotorParamSrv.response.txSuccess);
    else
    {
        ROS_ERROR("Failed to call service.");
        return -1;
    }
    ros::Duration(0.5).sleep();

    // Home all motors
    homeAllMotorsClient.call(emptySrv);
    ros::Duration(3).sleep();

    // Test arm wave
    testArmWave(setMotorGoalPositionInRadClient);
    ros::Duration(1.0).sleep();

    // Home all motors
    homeAllMotorsClient.call(emptySrv);
    ros::Duration(3).sleep();

    // Turn off all torques
    sendToAXSrv.request.dxlID = 254;
    sendToAXSrv.request.address = AX12_TORQUE_ENABLE;
    sendToAXSrv.request.value = 0;
    if (sendToAXClient.call(sendToAXSrv))
        ROS_INFO("TX success: %d", sendToAXSrv.response.txSuccess);
    else
    {
        ROS_ERROR("Failed to call service.");
        return -1;
    }
    ros::Duration(0.5).sleep();

    return 0;
}

