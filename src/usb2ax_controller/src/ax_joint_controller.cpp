#include "ax_joint_controller.h"
//#include "usb2ax/dynamixel.h"
#include "usb2ax/dynamixel_syncread.h"
#include "ax12ControlTableMacros.h"
#include "axs1ControlTableMacros.h"
#include <sstream>

// Note: USB2AX uses a different dxl_hal.c file than the Robotis' Dynamixel SDK for Linux.
// The Dynamixel SDK assumes the interface is FTDI-based, and thus searches a device named ttyUSBx,
// while the USB2AX uses the integrated CDC/ACM driver - which names the device ttyACMx.
// The second problem is that after opening the device, the Dynamixel SDK tries to set parameters which do
// not exist in the CDC/ACM driver.
// For more information see:
// http://www.xevelabs.com/doku.php?id=product:usb2ax:faq#qdynamixel_sdkhow_do_i_use_it_with_the_usb2ax

#define NUM_OF_MOTORS 18
#define FLOAT_PRECISION_THRESH 0.00001

// IDs 1-99 are assumed used for motors
// IDs 100-253 are assumed used for sensors
// Use ID 254 to broadcast to all motors


int main(int argc, char **argv)
{
    JointController jointController;

    // Argument 1: Device index, default = 0
    // Argument 2: Baud number, default = 1
    if (argc >= 2)
    {
        int val;
        std::stringstream iss(argv[1]);
        if (iss >> val)
            jointController.setDeviceIndex(val);
    }
    if (argc >= 3)
    {
        int val;
        std::istringstream iss(argv[2]);
        if (iss >> val)
            jointController.setBaudNum(val);
    }

    jointController.init();

    // Setup ROS
    ros::init(argc, argv, "ax_joint_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(1000);  // Hz

    // Joint state publisher
    jointController.pub = n.advertise<sensor_msgs::JointState>("ax_joint_states", 1000);

    // Services
    ros::ServiceServer getFromAXService =
            n.advertiseService("GetFromAX",
                               &JointController::getFromAX, &jointController);
    ros::ServiceServer sendtoAXService =
            n.advertiseService("SendToAX",
                               &JointController::sendToAX, &jointController);
    ros::ServiceServer sendSynctoAXService =
            n.advertiseService("SendSyncToAX",
                               &JointController::sendSyncToAX, &jointController);
    ros::ServiceServer getMotorCurrentPositionInRadService =
            n.advertiseService("GetMotorCurrentPositionInRad",
                               &JointController::getMotorCurrentPositionInRad, &jointController);
    ros::ServiceServer setMotorGoalPositionInRadService =
            n.advertiseService("SetMotorGoalPositionInRad",
                               &JointController::setMotorGoalPositionInRad, &jointController);
    ros::ServiceServer getMotorCurrentSpeedInRadPerSecService =
            n.advertiseService("GetMotorCurrentSpeedInRadPerSec",
                               &JointController::getMotorCurrentSpeedInRadPerSec, &jointController);
    ros::ServiceServer setMotorGoalSpeedInRadPerSecService =
            n.advertiseService("SetMotorGoalSpeedInRadPerSec",
                               &JointController::setMotorGoalSpeedInRadPerSec, &jointController);
    ros::ServiceServer getMotorCurrentTorqueInDecimalService =
            n.advertiseService("GetMotorCurrentTorqueInDecimal",
                               &JointController::getMotorCurrentTorqueInDecimal, &jointController);
    ros::ServiceServer setMotorMaxTorqueInDecimalService =
            n.advertiseService("SetMotorMaxTorqueInDecimal",
                               &JointController::setMotorMaxTorqueInDecimal, &jointController);
    ros::ServiceServer homeAllMotorsService =
            n.advertiseService("HomeAllMotors",
                               &JointController::homeAllMotors, &jointController);


//    // Tests
//    jointController.testSending();
//    ros::Duration(0.5).sleep();

//    jointController.testSendingSync();
//    ros::Duration(0.5).sleep();

//    jointController.testValueConversions();
//    ros::Duration(0.5).sleep();


//    // Set low torque
//    jointController.setAllMotorsMaxTorqueInDecimal(0.8);
//    ros::Duration(0.5).sleep();

//    // Set slow speed
//    jointController.setAllMotorsGoalSpeedInRadPerSec(1.0);
//    ros::Duration(0.5).sleep();

//    // Home all motors
//    jointController.homeAllMotors(0.0);
//    ros::Duration(3).sleep();

//    // Test arm wave
//    jointController.testArmWave();
//    ros::Duration(1.0).sleep();

//    // Home all motors
//    jointController.homeAllMotors(0.0);
//    ros::Duration(3).sleep();

//    // Turn off all torques
//    jointController.setAllMotorsTorqueEnabled(false);
//    ros::Duration(0.5).sleep();


//    // USB2AX sync_read test
//    int ids[18];
//    for (int dxlID = 1; dxlID <= NUM_OF_MOTORS; ++dxlID)
//        ids[dxlID-1] = dxlID;
//    jointController.sync_read(ids);
    jointController.getAllMotorPositions();


    while (ros::ok())
    {
        //jointController.run();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


JointController::JointController() :
    deviceIndex(0),
    baudNum(1),
    numOfConnectedMotors(0)
{
    connectedMotors.resize(NUM_OF_MOTORS+1);
    for (std::vector<bool>::iterator it = connectedMotors.begin(); it != connectedMotors.end(); ++it)
        *it = false;

    // Joint 0 is ignored, so that joint numbers match servo IDs
    joint_state.name.resize(NUM_OF_MOTORS+1);
    joint_state.position.resize(NUM_OF_MOTORS+1);
    joint_state.velocity.resize(NUM_OF_MOTORS+1);
    joint_state.effort.resize(NUM_OF_MOTORS+1);

    positionOffsets.resize(NUM_OF_MOTORS+1);
    directionSign.resize(NUM_OF_MOTORS+1);
}


JointController::~JointController()
{

}


int JointController::init()
{
    // Initialise comms
    if( dxl_initialize(deviceIndex, baudNum) == 0 )
    {
        ROS_ERROR("Failed to open USB2AX.");
        return -1;
    }
    else
    {
        ROS_INFO("USB2AX opened successfully.");

        // Find motors with IDs 1-NUM_OF_MOTORS
        for (int dxlID = 1; dxlID <= NUM_OF_MOTORS; ++dxlID)
        {
            dxl_ping(dxlID);
            if(dxl_get_result() == COMM_RXSUCCESS)
            {
                connectedMotors[dxlID] = true;
                ++numOfConnectedMotors;
                ROS_INFO("Motor with ID %d connected.", dxlID);
            }                
        }

        ROS_INFO("%d motors connected.", numOfConnectedMotors);
        if (numOfConnectedMotors != NUM_OF_MOTORS)
            ROS_WARN("Number of motors should be %d.", NUM_OF_MOTORS);

        // Empty joint 0
        joint_state.name[0] ="0";
        joint_state.position[0] = 0.0;
        joint_state.velocity[0] = 0.0;
        joint_state.effort[0] = 0.0;
        positionOffsets[0] = 0.0;
        directionSign[0] = 1;

        // Right arm
        joint_state.name[1] = "right_shoulder_swing_joint";
        positionOffsets[1] = M_PI/2.0;
        directionSign[1] = 1;
        joint_state.name[3] = "right_shoulder_lateral_joint";
        positionOffsets[3] = M_PI/2.0;
        directionSign[3] = 1;
        joint_state.name[5] = "right_elbow_joint";
        positionOffsets[5] = 0.0;
        directionSign[5] = 1;

        // Left arm
        joint_state.name[2] ="left_shoulder_swing_joint";
        positionOffsets[2] = M_PI/2.0;
        directionSign[2] = -1;
        joint_state.name[4] ="left_shoulder_lateral_joint";
        positionOffsets[4] = M_PI/2.0;
        directionSign[4] = -1;
        joint_state.name[6] ="left_elbow_joint";
        positionOffsets[6] = 0.0;
        directionSign[6] = -1;

        // Right leg
        joint_state.name[7] ="right_hip_twist_joint";
        positionOffsets[7] = 0.0;
        directionSign[7] = 1;
        joint_state.name[9] ="right_hip_lateral_joint";
        positionOffsets[9] = 0.0;
        directionSign[9] = -1;
        joint_state.name[11] ="right_hip_swing_joint";
        positionOffsets[11] = 0.0;
        directionSign[11] = -1;
        joint_state.name[13] ="right_knee_joint";
        positionOffsets[13] = 0.0;
        directionSign[13] = 1;
        joint_state.name[15] ="right_ankle_swing_joint";
        positionOffsets[15] = 0.0;
        directionSign[15] = 1;
        joint_state.name[17] ="right_ankle_lateral_joint";
        positionOffsets[17] = 0.0;
        directionSign[17] = 1;

        // Left leg
        joint_state.name[8] ="left_hip_twist_joint";
        positionOffsets[8] = 0.0;
        directionSign[8] = -1;
        joint_state.name[10] ="left_hip_lateral_joint";
        positionOffsets[10] = 0.0;
        directionSign[10] = 1;
        joint_state.name[12] ="left_hip_swing_joint";
        positionOffsets[12] = 0.0;
        directionSign[12] = 1;
        joint_state.name[14] ="left_knee_joint";
        positionOffsets[14] = 0.0;
        directionSign[14] = -1;
        joint_state.name[16] ="left_ankle_swing_joint";
        positionOffsets[16] = 0.0;
        directionSign[16] = -1;
        joint_state.name[18] ="left_ankle_lateral_joint";
        positionOffsets[18] = 0.0;
        directionSign[18] = -1;

        // Reduce Return Delay Time to speed up comms
        setValue(254, AX12_RETURN_DELAY_TIME, 0);
    }
    return 0;
}


void JointController::run()
{
    joint_state.header.stamp = ros::Time::now();

    for (int dxlID = 1; dxlID <= NUM_OF_MOTORS; ++dxlID)
    {
        if (connectedMotors[dxlID])
        {
                joint_state.position[dxlID] = getMotorCurrentPositionInRad(dxlID);
                joint_state.velocity[dxlID] = getMotorCurrentSpeedInRadPerSec(dxlID);
                joint_state.effort[dxlID] = getMotorCurrentTorqueInDecimal(dxlID);
        }
    }

    pub.publish(joint_state);
}


bool JointController::testSending()
{
    int dxlID = 1;
    int address = 30;
    int value = 100;
    ROS_INFO("Success: %d", setValue(dxlID, address, value));
    return 0;
}


bool JointController::testSendingSync()
{
    std_msgs::UInt16MultiArray dxlIDs;
    dxlIDs.layout.dim.resize(1);
    dxlIDs.layout.dim[0].label = "ids";
    dxlIDs.layout.dim[0].size = 3;          // Number of motors
    dxlIDs.layout.dim[0].stride = 3;        // Size of array (motors)
    dxlIDs.layout.data_offset = 0;
    dxlIDs.data.resize(3);                  // Size of array (motors)
    dxlIDs.data[0] = 1;
    dxlIDs.data[1] = 2;
    dxlIDs.data[2] = 3;

    int controlTableStartAddr = 30;

    std_msgs::UInt16MultiArray vals;
    vals.layout.dim.resize(2);
    vals.layout.dim[0].label = "bytes";
    vals.layout.dim[0].size = 3;            // Number of values
    vals.layout.dim[0].stride = 9;          // Size of array (values*motors)
    vals.layout.dim[1].label = "motors";
    vals.layout.dim[1].size = 3;            // Number of motors
    vals.layout.dim[1].stride = 3;          // Number of values
    vals.layout.data_offset = 0;
    vals.data.resize(9);                    // Size of array (values*motors)
    vals.data[0] = 100;
    vals.data[1] = 300;
    vals.data[2] = 512;
    vals.data[3] = 100;
    vals.data[4] = 300;
    vals.data[5] = 512;
    vals.data[6] = 100;
    vals.data[7] = 300;
    vals.data[8] = 512;

    std_msgs::UInt16MultiArray isWord;
    isWord.layout.dim.resize(1);
    isWord.layout.dim[0].label = "isWord";
    isWord.layout.dim[0].size = 3;          // Number of values
    isWord.layout.dim[0].stride = 3;        // Size of array (motors)
    isWord.layout.data_offset = 0;
    isWord.data.resize(3);                  // Size of array (motors)
    isWord.data[0] = true;
    isWord.data[1] = true;
    isWord.data[2] = true;

    ROS_INFO("Success: %d", setSyncValues(dxlIDs, controlTableStartAddr, vals, isWord));
    return 0;
}


bool JointController::testValueConversions()
{
    int posInAx = 512;
    ROS_INFO("Test position in AX value: \t\t\t%d", posInAx);
    float posInRad = axPositionToRad(posInAx);
    ROS_INFO("Test position converted to rad: \t\t%g", posInRad);
    posInAx = radToAxPosition(posInRad);
    ROS_INFO("Test position converted back to AX value: \t%d", posInAx);
    ROS_INFO("----");

    int speedInAx = 1023;
    ROS_INFO("Test speed in AX value: \t\t\t%d", speedInAx);
    float speedInRadPerSec = axSpeedToRadPerSec(speedInAx);
    ROS_INFO("Test speed converted to rad/sec: \t\t%g", speedInRadPerSec);
    speedInAx = radPerSecToAxSpeed(speedInRadPerSec);
    ROS_INFO("Test speed converted back to AX value: \t\t%d", speedInAx);
    ROS_INFO("----");

    int torqueInAx = 1023;
    ROS_INFO("Test torque in AX value: \t\t\t%d", torqueInAx);
    float torqueInPerc = axTorqueToDecimal(torqueInAx);
    ROS_INFO("Test torque converted to percentage: \t\t%g", torqueInPerc);
    torqueInAx = decimalToAxTorque(torqueInPerc);
    ROS_INFO("Test torque converted back to AX value: \t%d", torqueInAx);
    ROS_INFO("----");

    posInRad = 0.0;
    ROS_INFO("Test position in rad: \t\t\t%g", posInRad);
    posInAx = radToAxPosition(directionSign[0]*(posInRad - positionOffsets[0]));
    ROS_INFO("Test position converted to AX value: \t%d", posInAx);
    ROS_INFO("----");

    return 0;
}


void JointController::testArmWave()
{
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
    float samplingInterval = 0.01;
    int samples = (int)ceil(runTime/samplingInterval);
    int i2 = (int)ceil(delayTime/samplingInterval);

	// Initial position - Arms outstreched
    setMotorGoalPositionInRad(3, p3);
    setMotorGoalPositionInRad(5, p5);
    setMotorGoalPositionInRad(4, p4 + fi);
    setMotorGoalPositionInRad(6, p6 + fi);
    ros::Duration(2).sleep();
	
    // Wave
    ROS_INFO("Amplitude: %g", A);
    ROS_INFO("Runtime: %g", runTime);
    ROS_INFO("Frequency 1: %g", f1);
    ROS_INFO("Frequency 2: %g", f2);
    ROS_INFO("Arm 2 delay: %g", delayTime);
    ROS_INFO("Phase diff.: %g", fi);
    ROS_INFO("Sampling interval: %g", samplingInterval);
    ROS_INFO("Samples: %d", samples);
    ROS_INFO("i2: %d", i2);
    ROS_INFO("y1\t y2\t y3\t y4");
    float y1, y2, y3, y4;
    for (int i = 0; i < samples; ++i)
    {
        //ROS_INFO("i: %d", i);

        float t = i*samplingInterval;

        // Right arm
        y1 = A*sin( 2.0*M_PI*f1*t );
        y2 = A*sin( 2.0*M_PI*f2*t );
        setMotorGoalPositionInRad(3, p3 + y1);
        setMotorGoalPositionInRad(5, p5 + y2);

        if ( i > i2 )
        {
            // Left arm
            y3 = A*sin( 2.0*M_PI*f1*(t-delayTime) + fi );
            y4 = A*sin( 2.0*M_PI*f2*(t-delayTime) + fi );
            setMotorGoalPositionInRad(4, p4 + y3);
            setMotorGoalPositionInRad(6, p6 + y4);
            ROS_INFO("%g\t %g\t %g\t %g", y1, y2, y3, y4);
        }
        else
            ROS_INFO("%g\t %g\t", y1, y2);

        ros::Duration(samplingInterval).sleep();
    }
}


float JointController::getMotorCurrentPositionInRad(int dxlID)
{
    int readVal;
    if ( getValue( dxlID, AX12_PRESENT_POSITION_L, readVal ) == 0 )
        return directionSign[dxlID]*(axPositionToRad(readVal)) + positionOffsets[dxlID];
    else
        return 0.0;
}


void JointController::setMotorGoalPositionInRad(int dxlID, float pos)
{
    ROS_DEBUG("Offset: %g", positionOffsets[dxlID]);
    ROS_DEBUG("Direction sign: %d", directionSign[dxlID]);
    ROS_DEBUG("Value: %d", radToAxPosition(directionSign[dxlID]*(pos - positionOffsets[dxlID])));
    ROS_DEBUG("----");
    setValue( dxlID, AX12_GOAL_POSITION_L, radToAxPosition(directionSign[dxlID]*(pos - positionOffsets[dxlID])) );
}


float JointController::getMotorCurrentSpeedInRadPerSec(int dxlID)
{
    int readVal;
    if ( getValue( dxlID, AX12_PRESENT_SPEED_L, readVal ) == 0 )
        return axSpeedToRadPerSec(readVal);
    else
        return 0.0;
}


void JointController::setMotorGoalSpeedInRadPerSec(int dxlID, float pos)
{
    setValue( dxlID, AX12_MOVING_SPEED_L, radPerSecToAxSpeed(pos) );
}


float JointController::getMotorCurrentTorqueInDecimal(int dxlID)
{
    int readVal;
    if ( getValue( dxlID, AX12_PRESENT_LOAD_L, readVal ) == 0 )
        return axTorqueToDecimal(readVal);
    else
        return 0.0;
}


void JointController::setMotorMaxTorqueInDecimal(int dxlID, float pos)
{
    setValue( dxlID, AX12_TORQUE_LIMIT_L, decimalToAxTorque(pos) );
}


void JointController::getAllMotorPositions()
{
    std_msgs::UInt16MultiArray dxlIDs;
    dxlIDs.layout.dim.resize(1);
    dxlIDs.layout.dim[0].label = "ids";
    dxlIDs.layout.dim[0].size = numOfConnectedMotors;
    dxlIDs.layout.dim[0].stride = numOfConnectedMotors;
    dxlIDs.layout.data_offset = 0;
    dxlIDs.data.resize(numOfConnectedMotors);

    std_msgs::UInt16MultiArray vals;
    vals.layout.dim.resize(2);
    vals.layout.dim[0].label = "bytes";
    vals.layout.dim[0].size = 1;
    vals.layout.dim[0].stride = numOfConnectedMotors*1;
    vals.layout.dim[1].label = "motors";
    vals.layout.dim[1].size = numOfConnectedMotors;
    vals.layout.dim[1].stride = 1;
    vals.layout.data_offset = 0;
    vals.data.resize(numOfConnectedMotors*1);

    std_msgs::UInt16MultiArray isWord;
    isWord.layout.dim.resize(1);
    isWord.layout.dim[0].label = "isWord";
    isWord.layout.dim[0].size = 1;
    isWord.layout.dim[0].stride = 1;
    isWord.layout.data_offset = 0;
    isWord.data.resize(1);
    isWord.data[0] = true;

    for (int dxlID = 1; dxlID <= numOfConnectedMotors; ++dxlID)
    {
        if (connectedMotors[dxlID])
            dxlIDs.data[dxlID-1] = dxlID;
    }

    getSyncValues(dxlIDs, AX12_GOAL_POSITION_L, vals, isWord);
}


void JointController::homeAllMotors()
{
    std_msgs::UInt16MultiArray dxlIDs;
    dxlIDs.layout.dim.resize(1);
    dxlIDs.layout.dim[0].label = "ids";
    dxlIDs.layout.dim[0].size = numOfConnectedMotors;
    dxlIDs.layout.dim[0].stride = numOfConnectedMotors;
    dxlIDs.layout.data_offset = 0;
    dxlIDs.data.resize(numOfConnectedMotors);

    std_msgs::UInt16MultiArray vals;
    vals.layout.dim.resize(2);
    vals.layout.dim[0].label = "bytes";
    vals.layout.dim[0].size = 1;
    vals.layout.dim[0].stride = numOfConnectedMotors*1;
    vals.layout.dim[1].label = "motors";
    vals.layout.dim[1].size = numOfConnectedMotors;
    vals.layout.dim[1].stride = 1;
    vals.layout.data_offset = 0;
    vals.data.resize(numOfConnectedMotors*1);

    std_msgs::UInt16MultiArray isWord;
    isWord.layout.dim.resize(1);
    isWord.layout.dim[0].label = "isWord";
    isWord.layout.dim[0].size = 1;
    isWord.layout.dim[0].stride = 1;
    isWord.layout.data_offset = 0;
    isWord.data.resize(1);
    isWord.data[0] = true;

    for (int dxlID = 1; dxlID <= numOfConnectedMotors; ++dxlID)
    {
        if (connectedMotors[dxlID])
        {
            dxlIDs.data[dxlID-1] = dxlID;
            vals.data[dxlID-1] = radToAxPosition(directionSign[dxlID]*(0.0 - positionOffsets[dxlID]));
        }
    }

    setSyncValues(dxlIDs, AX12_GOAL_POSITION_L, vals, isWord);
}


bool JointController::getFromAX(
        usb2ax_controller::GetFromAX::Request &req, usb2ax_controller::GetFromAX::Response &res)
{
    int readVal;
    if ( getValue( req.dxlID, req.address, readVal ) == 0 )
    {
        res.value = readVal;
        res.rxSuccess = true;
        return true;
    }
    else
    {
        res.value = 0;
        res.rxSuccess = false;
        return false;
    }
}


bool JointController::getSyncFromAX(
        usb2ax_controller::GetSyncFromAX::Request &req, usb2ax_controller::GetSyncFromAX::Response &res)
{
    if (!getSyncValues(req.dxlIDs, req.startAddress, res.values, req.isWord))
    {
        res.txSuccess = true;
        return true;
    }
    else
    {
        res.txSuccess = false;
        return false;
    }
}


bool JointController::sendToAX(
        usb2ax_controller::SendToAX::Request &req, usb2ax_controller::SendToAX::Response &res)
{
    if (!setValue(req.dxlID, req.address, req.value))
    {
        res.txSuccess = true;
        return true;
    }
    else
    {
        res.txSuccess = false;
        return false;
    }
}


bool JointController::sendSyncToAX(
        usb2ax_controller::SendSyncToAX::Request &req, usb2ax_controller::SendSyncToAX::Response &res)
{
    // Example: IDs 1, 2, 3: Set position 100, speed 300, torque limit 512
    // controlTableStartAddr = 30 (Goal Position L)
    // dxlIDs:  1             | 2             | 3
    // vals:    100, 300, 512 | 100, 300, 512 | 100, 300, 512
    // isWord:    1,   1,   1
    //
    // rosservice command line example
    // With strings:
    //rosservice call /SendSyncToAX
    //    '[[[["ids", 3, 3]], 0], [1, 2, 3]]' '30'
    //    '[[[["bytes", 3, 9], ["motors", 3, 3]], 0], [100, 300, 512, 100, 300, 512, 100, 300, 512]]'
    //    '[[[["isWord", 3, 3]], 0], [1, 1, 1]]'
    //
    // With YAML dictionaries:
    //rosservice call /SendSyncToAX
    //    '{ layout: {dim: [{label: "ids", size: 3, stride: 3}], data_offset: 0}, data: [1, 2, 3] }' '30'
    //    '{ layout: {dim: [{label: "bytes", size: 3, stride: 9},
    //                      {label: "motors", size: 3, stride: 3}], data_offset: 0},
    //                data: [100, 300, 512, 100, 300, 512, 100, 300, 512] }'
    //    '{ layout: {dim: [{label: "isWord", size: 3, stride: 3}], data_offset: 0}, data: [1, 1, 1] }'

    if (!setSyncValues(req.dxlIDs, req.startAddress, req.values, req.isWord))
    {
        res.txSuccess = true;
        return true;
    }
    else
    {
        res.txSuccess = false;
        return false;
    }
}


bool JointController::getMotorCurrentPositionInRad(
        usb2ax_controller::GetMotorParam::Request &req,
        usb2ax_controller::GetMotorParam::Response &res)
{
    res.value = getMotorCurrentPositionInRad(req.dxlID);
    res.txSuccess = true;
    return true;
}


bool JointController::setMotorGoalPositionInRad(
        usb2ax_controller::SetMotorParam::Request &req,
        usb2ax_controller::SetMotorParam::Response &res)
{
    setMotorGoalPositionInRad(req.dxlID, req.value);
    res.txSuccess = true;
    return true;
}


bool JointController::getMotorCurrentSpeedInRadPerSec(
        usb2ax_controller::GetMotorParam::Request &req,
        usb2ax_controller::GetMotorParam::Response &res)
{
    res.value = getMotorCurrentSpeedInRadPerSec(req.dxlID);
    res.txSuccess = true;
    return true;
}


bool JointController::setMotorGoalSpeedInRadPerSec(
        usb2ax_controller::SetMotorParam::Request &req,
        usb2ax_controller::SetMotorParam::Response &res)
{
    setMotorGoalSpeedInRadPerSec(req.dxlID, req.value);
    res.txSuccess = true;
    return true;
}


bool JointController::getMotorCurrentTorqueInDecimal(
        usb2ax_controller::GetMotorParam::Request &req,
        usb2ax_controller::GetMotorParam::Response &res)
{
    res.value = getMotorCurrentTorqueInDecimal(req.dxlID);
    res.txSuccess = true;
    return true;
}


bool JointController::setMotorMaxTorqueInDecimal(
        usb2ax_controller::SetMotorParam::Request &req,
        usb2ax_controller::SetMotorParam::Response &res)
{
    setMotorMaxTorqueInDecimal(req.dxlID, req.value);
    res.txSuccess = true;
    return true;
}


bool JointController::homeAllMotors(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    homeAllMotors();
    return true;
}


bool JointController::getValue(int dxlID, int controlTableAddr, int& val)
{
    // Motor
    if ( (1 <= dxlID) and (dxlID < 100) )
    {
        // Read word
        switch (controlTableAddr)
        {
        case AX12_MODEL_NUMBER_L:
        case AX12_CW_ANGLE_LIMIT_L:
        case AX12_CCW_ANGLE_LIMIT_L:
        case AX12_MAX_TORQUE_L:
        case AX12_GOAL_POSITION_L:
        case AX12_MOVING_SPEED_L:
        case AX12_TORQUE_LIMIT_L:
        case AX12_PRESENT_POSITION_L:
        case AX12_PRESENT_SPEED_L:
        case AX12_PRESENT_LOAD_L:
        case AX12_PUNCH_L:
        {
            val = dxl_read_word(dxlID, controlTableAddr);
            break;
        }
        // Read byte
        default:
        {
            val = dxl_read_byte(dxlID, controlTableAddr);
            break;
        }
        }
    }
    // Sensor
    else if (dxlID >= 100)
    {
        // Read word
        switch (controlTableAddr)
        {
        case AXS1_MODEL_NUMBER_L:
        case AXS1_SOUND_DETECTED_TIME_L:
        case AXS1_REMOCON_RX_DATA_L:
        case AXS1_REMOCON_TX_DATA_L:
        {
            val = dxl_read_word(dxlID, controlTableAddr);
            break;
        }
        // Read byte
        default:
        {
            val = dxl_read_byte(dxlID, controlTableAddr);
            break;
        }
        }
    }
    else
        return -1;

    int CommStatus = dxl_get_result();
    if (CommStatus == COMM_RXSUCCESS)
    {
        //ROS_DEBUG("Value received: %d", val);
        printErrorCode();
        return 0;
    }
    else
    {
        printCommStatus(CommStatus);
        return -1;
    }
}


bool JointController::setValue(int dxlID, int controlTableAddr, int val)
{
    // Motor
    if ( (1 <= dxlID) and (dxlID < 100) or (dxlID == 254) )
    {
        switch (controlTableAddr)
        {
        case AX12_MODEL_NUMBER_L:
        case AX12_CW_ANGLE_LIMIT_L:
        case AX12_CCW_ANGLE_LIMIT_L:
        case AX12_MAX_TORQUE_L:
        case AX12_GOAL_POSITION_L:
        case AX12_MOVING_SPEED_L:
        case AX12_TORQUE_LIMIT_L:
        case AX12_PRESENT_POSITION_L:
        case AX12_PRESENT_SPEED_L:
        case AX12_PRESENT_LOAD_L:
        case AX12_PUNCH_L:
        {
            // 2 bytes
            dxl_write_word(dxlID, controlTableAddr, val);
            break;
        }
        default:
        {
            // 1 byte
            dxl_write_byte(dxlID, controlTableAddr, val);
            break;
        }
        }
    }
    // Sensor
    else if (dxlID >= 100)
    {
        switch (controlTableAddr)
        {
        case AXS1_MODEL_NUMBER_L:
        case AXS1_SOUND_DETECTED_TIME_L:
        case AXS1_REMOCON_RX_DATA_L:
        case AXS1_REMOCON_TX_DATA_L:
        {
            // 2 bytes
            dxl_write_word(dxlID, controlTableAddr, val);
            break;
        }
        default:
        {
            // 1 byte
            dxl_write_byte(dxlID, controlTableAddr, val);
            break;
        }
        }
    }
    else
        return -1;

    // No return Status Packet from a broadcast command
    if (dxlID == 254)
        return 0;
    else
    {
        int CommStatus = dxl_get_result();
        if (CommStatus == COMM_RXSUCCESS)
        {
            //ROS_DEBUG("Value sent: %d", val);
            printErrorCode();
            return 0;
        }
        else
        {
            printCommStatus(CommStatus);
            return -1;
        }
    }
}


void JointController::sync_read(int ids[18])
{
    dxl_sync_read_start( AX12_PRESENT_POSITION_L, 2 );
    for (int i = 0; i < NUM_OF_MOTORS; i++ )
        dxl_sync_read_push_id( ids[i] );
    dxl_sync_read_send();
    int CommStatus = dxl_get_result();
    if( CommStatus == COMM_RXSUCCESS )
    {
        for (int i = 0; i < NUM_OF_MOTORS; i++ )
            printf( "%i=%i, ", ids[i] , dxl_sync_read_pop_word());
    }
    else
        printCommStatus(CommStatus);
    printf("\n");
}


int JointController::getSyncValues(
        std_msgs::UInt16MultiArray dxlIDs, int controlTableStartAddr,
        std_msgs::UInt16MultiArray &vals, std_msgs::UInt16MultiArray isWord)
{
    int numOfMotors = dxlIDs.data.size();

    if ( numOfMotors == 0 )
    {
        ROS_ERROR("No motors specified.");
        return -1;
    }

    if ( isWord.data.size() != (vals.data.size()/numOfMotors) )
    {
        ROS_ERROR("Input data size mismatch.");
        return -1;
    }

    // Length of data for each servo
    int dataLength = 0;

    for (std::vector<u_int16_t>::const_iterator it = isWord.data.begin(); it != isWord.data.end(); ++it)
    {
        if (*it == false)
            ++dataLength;
        else
            dataLength = dataLength + 2;
    }

    // Generate sync_read command
    dxl_sync_read_start(controlTableStartAddr, dataLength);
    for (int i = 0; i < numOfMotors; i++ )
        dxl_sync_read_push_id(dxlIDs.data[i]);
    dxl_sync_read_send();

    int CommStatus = dxl_get_result();
    if (CommStatus == COMM_RXSUCCESS)
    {
        for (int i = 0; i < numOfMotors; ++i)
        {
            for (int j = 0; j < isWord.data.size(); ++j)
            {
                int res = dxl_sync_read_pop_word();
                ROS_INFO( "%d=%d, ", dxlIDs.data[i], res);
                vals.data[vals.layout.dim[1].stride*i + j] = res;
            }
        }
        printErrorCode();
        return 0;
    }
    else
    {
        printCommStatus(CommStatus);
        return -1;
    }
}


int JointController::setSyncValues(
        std_msgs::UInt16MultiArray dxlIDs, int controlTableStartAddr,
        std_msgs::UInt16MultiArray vals, std_msgs::UInt16MultiArray isWord)
{
    int numOfMotors = dxlIDs.data.size();

    if ( numOfMotors == 0 )
    {
        ROS_ERROR("No motors specified.");
        return -1;
    }

    if ( isWord.data.size() != (vals.data.size()/numOfMotors) )
    {
        ROS_ERROR("Input data size mismatch.");
        return -1;
    }

    // Length of data for each servo
    int dataLength = 0;

    for (std::vector<u_int16_t>::const_iterator it = isWord.data.begin(); it != isWord.data.end(); ++it)
    {
        if (*it == false)
            ++dataLength;
        else
            dataLength = dataLength + 2;
    }

    // Make sync_write packet
    ROS_DEBUG( "Packet" );
    ROS_DEBUG( "ID:\t\t\t %d", BROADCAST_ID );
    ROS_DEBUG( "Instr:\t\t\t %d", INST_SYNC_WRITE );
    ROS_DEBUG( "Param 0 (start addr):\t %d", controlTableStartAddr );
    ROS_DEBUG( "Param 1 (data length):\t %d", dataLength );
    dxl_set_txpacket_id(BROADCAST_ID);
    dxl_set_txpacket_instruction(INST_SYNC_WRITE);
    dxl_set_txpacket_parameter(0, controlTableStartAddr);
    dxl_set_txpacket_parameter(1, dataLength);

    int paramIndex = 2;
    for (int i = 0; i < numOfMotors; ++i)
    {
        ROS_DEBUG( "Param %d (dxl %d):\t %d", paramIndex, i, dxlIDs.data[i] );
        dxl_set_txpacket_parameter( paramIndex++, dxlIDs.data[i] );
        for (int j = 0; j < isWord.data.size(); ++j)
        {
            ROS_DEBUG( "Value: %d", (vals.data[vals.layout.dim[1].stride*i + j]) );
            if (isWord.data[j])
            {
                // 2 bytes
                ROS_DEBUG( "Param %d (data %dL):\t %d", paramIndex, j,
                           dxl_get_lowbyte((int)(vals.data[vals.layout.dim[1].stride*i + j])) );
                ROS_DEBUG( "Param %d (data %dH):\t %d", paramIndex + 1, j,
                           dxl_get_highbyte((int)(vals.data[vals.layout.dim[1].stride*i + j])) );
                dxl_set_txpacket_parameter(
                            paramIndex++, dxl_get_lowbyte((int)(vals.data[vals.layout.dim[1].stride*i + j])) );
                dxl_set_txpacket_parameter(
                            paramIndex++, dxl_get_highbyte((int)(vals.data[vals.layout.dim[1].stride*i + j])) );
            }
            else
            {
                // 1 byte
                ROS_DEBUG( "Param %d (data %d):\t %d", paramIndex, j,
                           dxl_get_highbyte((int)(vals.data[vals.layout.dim[1].stride*i + j])) );
                dxl_set_txpacket_parameter(
                            paramIndex++, (int)(vals.data[vals.layout.dim[1].stride*i + j]) );
            }
            ROS_DEBUG("--");
        }
    }

    ROS_DEBUG( "Length:\t\t\t %d", (dataLength + 1)*numOfMotors + 4 );
    dxl_set_txpacket_length( (dataLength + 1)*numOfMotors + 4 );

    dxl_txrx_packet();

    int CommStatus = dxl_get_result();
    if (CommStatus == COMM_RXSUCCESS)
    {
        printErrorCode();
        return 0;
    }
    else
    {
        printCommStatus(CommStatus);
        return -1;
    }
}


void JointController::printCommStatus(int CommStatus)
{
    switch(CommStatus)
    {
    case COMM_TXFAIL:
    {
        ROS_ERROR("COMM_TXFAIL: Failed to transmit instruction packet!");
        break;
    }
    case COMM_TXERROR:
    {
        ROS_ERROR("COMM_TXERROR: Incorrect instruction packet!");
        break;
    }
    case COMM_RXFAIL:
    {
        ROS_ERROR("COMM_RXFAIL: Failed to get status packet from device!");
        break;
    }
    case COMM_RXWAITING:
    {
        ROS_ERROR("COMM_RXWAITING: Now receiving status packet!");
        break;
    }
    case COMM_RXTIMEOUT:
    {
        ROS_ERROR("COMM_RXTIMEOUT: There is no status packet!");
        break;
    }
    case COMM_RXCORRUPT:
    {
        ROS_ERROR("COMM_RXCORRUPT: Incorrect status packet!");
        break;
    }
    default:
    {
        ROS_ERROR("Unknown error code!");
        break;
    }
    }
}


void JointController::printErrorCode()
{
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
        ROS_ERROR("Input voltage error!");

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
        ROS_ERROR("Angle limit error!");

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
        ROS_ERROR("Overheat error!");

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
        ROS_ERROR("Out of range error!");

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
        ROS_ERROR("Checksum error!");

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
        ROS_ERROR("Overload error!");

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
        ROS_ERROR("Instruction code error!");
}


float JointController::axPositionToRad(int oldValue)
{
    // Convert AX-12 position to rads
    // ~0.2933 degrees per unit -> ~0.0051 rads per unit
    // Position range: 0..1023 -> 0..300 degrees -> 0..5.236 rad
    // Convert to -150..150 degrees -> -2.618..2.618 rad
    //
    // 1 deg = pi/180 rad
    //
    //int oldMin = 0;
    //int oldMax = 1023;
    //float newMin = -150.0*M_PI/180.0;
    //float newMax = 150.0*M_PI/180.0;
    //float oldRange = oldMax - oldMin;
    //float newRange = newMax - newMin;
    //float newValue = ((oldValue & 0x3FF) - oldMin)*newRange/oldRange + newMin;  // Bits 0-9
    //float newValue = (oldValue & 0x3FF)*0.0051 - 512*0.0051;  // Bits 0-9
    float newValue = ((oldValue & 0x3FF) - 512)*0.0051;  // Bits 0-9
    return newValue;
}


int JointController::radToAxPosition(float oldValue)
{
    // Convert rads to AX-12 position
    //if ( ((-150.0*M_PI/180.0 - FLOAT_PRECISION_THRESH) <= oldValue) and
    //     (oldValue <= (150.0*M_PI/180.0 + FLOAT_PRECISION_THRESH)) )
    //{
        //float oldMin = -150.0*M_PI/180.0;
        //float oldMax = 150.0*M_PI/180.0;
        //int newMin = 0;
        //int newMax = 1023;
        //float oldRange = oldMax - oldMin;
        //float newRange = newMax - newMin;
        //int newValue = round( (oldValue - oldMin)*newRange/oldRange + newMin );
        //return newValue;
    if ( ((-512*0.0051 - FLOAT_PRECISION_THRESH) <= oldValue) and
         (oldValue <= (512*0.0051 + FLOAT_PRECISION_THRESH)) )
    {
        //int newValue = round( (oldValue + 512*0.0051)/0.0051 );
        int newValue = round( oldValue/0.0051 + 512 );
        return newValue;
    }
    else
    {
        ROS_WARN("Value outside of valid input range, returning 0.");
        return 0;
    }
}


float JointController::axSpeedToRadPerSec(int oldValue)
{
    // Convert AX-12 speed to rads per sec
    // ~0.111 rpm per unit -> ~0.0116 rad/s per unit
    // Speed range:    0..1023 -> 0..113.553 rpm CCW -> 0..11.8668 rad/s CCW
    //              1024..2047 -> 0..113.553 rpm CW  -> 0..11.8668 rad/s CW
    //
    // v (Hz) = w (rad/s) / 2*pi
    // 1 rpm = 1/60 Hz ~= 0.1047 rad/s
    // 1 rad/s = 60/(2*pi) rpm ~= 9.5493 rpm ~= 0.1592 Hz
    //
    //int oldMin = 0;
    //int oldMax = 1023;
    //float newMin = 0.0;
    //float newMax = 1023*0.0116;
    //float oldRange = oldMax - oldMin;
    //float newRange = newMax - newMin;
    //float newValue = ((oldValue & 0x3FF) - oldMin)*newRange/oldRange + newMin;  // Bits 0-9
    float newValue = (oldValue & 0x3FF)*0.0116;  // Bits 0-9
    if ( (oldValue & 0x400) == 0x0 )  // Check bit 10
        return newValue;
    else
        return -newValue;
}


int JointController::radPerSecToAxSpeed(float oldValue)
{
    // Convert rads per sec to AX-12 speed
    //float oldMin = 0.0;
    //float oldMax = 1023*0.0116;
    //int newMin = 0;
    //int newMax = 1023;
    //float oldRange = oldMax - oldMin;
    //float newRange = newMax - newMin;
    //int newValue = round( (fabs(oldValue) - oldMin)*newRange/oldRange + newMin );
    int newValue = round( fabs(oldValue)/0.0116 );
    if ( (0.0 <= oldValue) and (oldValue <= (1023*0.0116 + FLOAT_PRECISION_THRESH)) )
        return newValue;
    else if ( ((-1023*0.0116 - FLOAT_PRECISION_THRESH) <= oldValue) and (oldValue < 0.0) )
        return newValue | 0x400;  // Set bit 10 to 1
    else
    {
        ROS_WARN("Value outside of valid input range, returning 0.");
        return 0;
    }
}


float JointController::axTorqueToDecimal(int oldValue)
{
    // Convert AX-12 torque to % torque
    // ~0.1% per unit
    // Torque range:    0-1023 -> 100% CCW
    //               1024-2047 -> 100% CW
    //
    //int oldMin = 0;
    //int oldMax = 1023;
    //float newMin = 0.0;
    //float newMax = 1023*0.001;
    //float oldRange = oldMax - oldMin;
    //float newRange = newMax - newMin;
    //float newValue = ((oldValue & 0x3FF) - oldMin)*newRange/oldRange + newMin;  // Bits 0-9
    float newValue = (oldValue & 0x3FF)*0.001;  // Bits 0-9
    if ( (oldValue & 0x400) == 0x0 )  // Check bit 10
        return newValue;
    else
        return -newValue;
}


int JointController::decimalToAxTorque(float oldValue)
{
    // Convert % torque to AX-12 torque
    //float oldMin = 0.0;
    //float oldMax = 1023*0.001;
    //int newMin = 0;
    //int newMax = 1023;
    //float oldRange = oldMax - oldMin;
    //float newRange = newMax - newMin;
    //int newValue = round( (fabs(oldValue) - oldMin)*newRange/oldRange + newMin );
    int newValue = round( fabs(oldValue)/0.001 );
    if ( (0.0 <= oldValue) and (oldValue <= (1023*0.001 + FLOAT_PRECISION_THRESH)) )
        return newValue;
    else if ( ((-1023*0.001 - FLOAT_PRECISION_THRESH) <= oldValue) and (oldValue < 0.0) )
        return newValue | 0x400;  // Set bit 10 to 1
    else
    {
        ROS_WARN("Value outside of valid input range, returning 0.");
        return 0;
    }
}

