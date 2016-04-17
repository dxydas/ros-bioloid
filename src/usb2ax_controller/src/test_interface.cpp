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
    ros::ServiceClient setMotorTorqueLimitInDecimalClient =
            n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorTorqueLimitInDecimal");
    ros::ServiceClient homeAllMotorsClient =
            n.serviceClient<std_srvs::Empty>("HomeAllMotors");

    usb2ax_controller::SendToAX sendToAXSrv;
    usb2ax_controller::SetMotorParam setMotorParamSrv;
    std_srvs::Empty emptySrv;

//    // Set max torque
//    setMotorParamSrv.request.dxlID = 254;
//    setMotorParamSrv.request.value = 0.8;
//    setMotorParamSrv.response.txSuccess = false;
//    if (setMotorTorqueLimitInDecimalClient.call(setMotorParamSrv))
//        ROS_INFO("TX success: %d", setMotorParamSrv.response.txSuccess);
//    else
//    {
//        ROS_ERROR("Failed to call service.");
//        return -1;
//    }
//    ros::Duration(0.5).sleep();

    // Set slow speed
    setMotorParamSrv.request.dxlID = 254;
    setMotorParamSrv.request.value = 1.0;
    setMotorParamSrv.response.txSuccess = false;
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
    sendToAXSrv.response.txSuccess = false;
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


// First tests from ax_joint_controller

//    usb2ax_controller::SendToAX::Request set_req;
//    usb2ax_controller::SendToAX::Response set_res;
//    usb2ax_controller::GetSyncFromAX::Request syncGet_req;
//    usb2ax_controller::GetSyncFromAX::Response syncGet_res;
//    usb2ax_controller::SetMotorParam::Request paramSet_req;
//    usb2ax_controller::SetMotorParam::Response paramSet_res;
//    std_srvs::Empty::Request empty_req;
//    std_srvs::Empty::Response empty_res;

//    jointController.testValueConversions();
//    ros::Duration(0.5).sleep();

//    // Set torque
//    paramSet_req.dxlID = 254;
//    paramSet_req.value = 0.8;
//    jointController.setMotorTorqueLimitInDecimal(paramSet_req, paramSet_res);
//    ros::Duration(0.5).sleep();

//    // Set slow speed
//    paramSet_req.dxlID = 254;
//    paramSet_req.value = 1.0;
//    jointController.setMotorGoalSpeedInRadPerSec(paramSet_req, paramSet_res);
//    ros::Duration(0.5).sleep();

//    jointController.testSending();
//    ros::Duration(2.0).sleep();

//    jointController.testSendingSync();
//    ros::Duration(2.0).sleep();

//    jointController.getAllMotorPositionsInRad(syncGet_req, syncGet_res);
//    ros::Duration(1.0).sleep();

//    // Home all motors
//    jointController.homeAllMotors(empty_req, empty_res);
//    ros::Duration(3.0).sleep();

//    // Test arm wave
//    jointController.testArmWave();
//    ros::Duration(1.0).sleep();

//    // Home all motors
//    jointController.homeAllMotors(empty_req, empty_res);
//    ros::Duration(3.0).sleep();

//    // Turn off all torques
//    set_req.dxlID = 254;
//    set_req.address = AX12_TORQUE_ENABLE;
//    set_req.value = 0;
//    jointController.sendToAX(set_req, set_res);
//    ros::Duration(0.5).sleep();


//bool JointController::testSending()
//{
//    usb2ax_controller::SendToAX::Request req;
//    usb2ax_controller::SendToAX::Response res;
//    req.dxlID = 1;
//    req.address = AX12_GOAL_POSITION_L;
//    req.value = 300;
//    ROS_INFO("Success: %d", sendToAX(req, res));
//    return 0;
//}


//bool JointController::testSendingSync()
//{
//    usb2ax_controller::SendSyncToAX::Request req;
//    usb2ax_controller::SendSyncToAX::Response res;

//    req.dxlIDs.resize(3);
//    req.dxlIDs[0] = 1;
//    req.dxlIDs[1] = 3;
//    req.dxlIDs[2] = 5;

//    req.startAddress = 30;

//    req.values.resize(9);
//    req.values[0] = 280;  // Position
//    req.values[1] = 100;  // Speed
//    req.values[2] = 512;  // Torque
//    req.values[3] = 350;  // Position
//    req.values[4] = 100;  // Speed
//    req.values[5] = 512;  // Torque
//    req.values[6] = 400;  // Position
//    req.values[7] = 100;  // Speed
//    req.values[8] = 512;  // Torque

//    req.isWord.resize(3);
//    req.isWord[0] = true;
//    req.isWord[1] = true;
//    req.isWord[2] = true;

//    ROS_INFO("Success: %d", sendSyncToAX(req, res));
//    return 0;
//}


//bool JointController::testValueConversions()
//{
//    int posInAx = 512;
//    ROS_INFO("Test position in AX value: \t\t\t%d", posInAx);
//    float posInRad = axPositionToRad(posInAx);
//    ROS_INFO("Test position converted to rad: \t\t%g", posInRad);
//    posInAx = radToAxPosition(posInRad);
//    ROS_INFO("Test position converted back to AX value: \t%d", posInAx);
//    ROS_INFO("----");

//    int speedInAx = 1023;
//    ROS_INFO("Test speed in AX value: \t\t\t%d", speedInAx);
//    float speedInRadPerSec = axSpeedToRadPerSec(speedInAx);
//    ROS_INFO("Test speed converted to rad/sec: \t\t%g", speedInRadPerSec);
//    speedInAx = radPerSecToAxSpeed(speedInRadPerSec);
//    ROS_INFO("Test speed converted back to AX value: \t\t%d", speedInAx);
//    ROS_INFO("----");

//    int torqueInAx = 1023;
//    ROS_INFO("Test torque in AX value: \t\t\t%d", torqueInAx);
//    float torqueInPerc = axTorqueToDecimal(torqueInAx);
//    ROS_INFO("Test torque converted to percentage: \t\t%g", torqueInPerc);
//    torqueInAx = decimalToAxTorque(torqueInPerc);
//    ROS_INFO("Test torque converted back to AX value: \t%d", torqueInAx);
//    ROS_INFO("----");

//    posInRad = 0.0;
//    ROS_INFO("Test position in rad: \t\t\t%g", posInRad);
//    posInAx = radToAxPosition(posInRad);
//    ROS_INFO("Test position converted to AX value: \t%d", posInAx);
//    ROS_INFO("----");

//    return 0;
//}


//void JointController::testArmWave()
//{
//    usb2ax_controller::SetMotorParam::Request req;
//    usb2ax_controller::SetMotorParam::Response res;

//    // Right arm: IDs 3, 5
//    // Left arm:  IDs 4, 6

//    float p3 = M_PI/2.0;
//    float p5 = 0.0;
//    float p4 = M_PI/2.0;
//    float p6 = 0.0;

//    float A = M_PI/4.0;
//    float runTime = 2.0;
//    float delayTime = runTime/2.0;
//    float f1 = 1.0/runTime;
//    float f2 = 2.0/runTime;
//    float fi = 0.0;//M_PI/4.0;
//    float samplingInterval = 0.01;
//    int samples = (int)ceil(runTime/samplingInterval);
//    int i2 = (int)ceil(delayTime/samplingInterval);

//    // Initial position - Arms outstreched
//    req.dxlID = 3;
//    req.value = p3;
//    setMotorGoalPositionInRad(req, res);
//    req.dxlID = 5;
//    req.value = p5;
//    setMotorGoalPositionInRad(req, res);
//    req.dxlID = 4;
//    req.value = p4 + fi;
//    setMotorGoalPositionInRad(req, res);
//    req.dxlID = 6;
//    req.value = p6 + fi;
//    setMotorGoalPositionInRad(req, res);
//    ros::Duration(2).sleep();

//    // Wave
//    ROS_INFO("Amplitude: %g", A);
//    ROS_INFO("Runtime: %g", runTime);
//    ROS_INFO("Frequency 1: %g", f1);
//    ROS_INFO("Frequency 2: %g", f2);
//    ROS_INFO("Arm 2 delay: %g", delayTime);
//    ROS_INFO("Phase diff.: %g", fi);
//    ROS_INFO("Sampling interval: %g", samplingInterval);
//    ROS_INFO("Samples: %d", samples);
//    ROS_INFO("i2: %d", i2);
//    ROS_INFO("y1\t y2\t y3\t y4");
//    float y1, y2, y3, y4;
//    for (int i = 0; i < samples; ++i)
//    {
//        //ROS_INFO("i: %d", i);

//        float t = i*samplingInterval;

//        // Right arm
//        y1 = A*sin( 2.0*M_PI*f1*t );
//        y2 = A*sin( 2.0*M_PI*f2*t );
//        req.dxlID = 3;
//        req.value = p3 + y1;
//        setMotorGoalPositionInRad(req, res);
//        req.dxlID = 5;
//        req.value = p5 + y2;
//        setMotorGoalPositionInRad(req, res);

//        if ( i > i2 )
//        {
//            // Left arm
//            y3 = A*sin( 2.0*M_PI*f1*(t-delayTime) + fi );
//            y4 = A*sin( 2.0*M_PI*f2*(t-delayTime) + fi );
//            req.dxlID = 4;
//            req.value = p4 + y3;
//            setMotorGoalPositionInRad(req, res);
//            req.dxlID = 6;
//            req.value = p6 + y4;
//            setMotorGoalPositionInRad(req, res);
//            ROS_INFO("%g\t %g\t %g\t %g", y1, y2, y3, y4);
//        }
//        else
//            ROS_INFO("%g\t %g\t", y1, y2);

//        ros::Duration(samplingInterval).sleep();
//    }
//}
