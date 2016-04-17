#include "ax_joint_controller.h"
#include <string>
#include <sstream>
#include "usb2ax/dynamixel_syncread.h"
#include "ax12ControlTableMacros.h"
#include "axs1ControlTableMacros.h"

#define NUM_OF_MOTORS 18
#define FLOAT_PRECISION_THRESH 0.00001

// IDs 1-99 are assumed used for motors
// IDs 100-253 are assumed used for sensors
// Use ID BROADCAST_ID (254) to broadcast to all motors


int main(int argc, char **argv)
{
    JointController jointController;

    // Argument 1: Write position controller values to motors, default = false
    // Argument 2: USB-to-serial device index, default = 0
    // Argument 3: USB-to-serial baud number, default = 1
    if (argc >= 2)
    {
        std::string val(argv[1]);
        if ( (val == "false") || (val == "0") )
            jointController.setPositionControlEnabled(false);
        else if ( (val == "true") || (val == "1") )
            jointController.setPositionControlEnabled(true);
        else
            std::cout << "Invalid first input argument, quitting.";
    }
    if (argc >= 3)
    {
        int val;
        std::istringstream iss(argv[2]);
        if (iss >> val)
            jointController.setDeviceIndex(val);
        else
            std::cout << "Invalid second input argument, quitting.";
    }
    if (argc >= 4)
    {
        int val;
        std::istringstream iss(argv[3]);
        if (iss >> val)
            jointController.setBaudNum(val);
        else
            std::cout << "Invalid third input argument, quitting.";
    }

    // Setup ROS
    ros::init(argc, argv, "ax_joint_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);  // Hz
    ROS_INFO("Controller node initialised.");
    ROS_INFO("Namespace: %s", n.getNamespace().c_str());

    // Joint state publisher
    jointController.jointStatePub = n.advertise<sensor_msgs::JointState>("ax_joint_states", 1000);

    // Goal joint state publisher
    jointController.goalJointStatePub = n.advertise<sensor_msgs::JointState>("ax_goal_joint_states", 1000);

    // Services
    ros::ServiceServer receiveFromAXService = n.advertiseService("ReceiveFromAX",
        &JointController::receiveFromAX, &jointController);
    ros::ServiceServer SendToAXService = n.advertiseService("SendToAX",
        &JointController::sendToAX, &jointController);
    //
    ros::ServiceServer receiveSyncFromAXService = n.advertiseService("ReceiveSyncFromAX",
        &JointController::receiveSyncFromAX, &jointController);
    ros::ServiceServer sendSyncToAXService = n.advertiseService("SendSyncToAX",
        &JointController::sendSyncToAX, &jointController);
    //
    ros::ServiceServer getMotorCurrentPositionInRadService = n.advertiseService("GetMotorCurrentPositionInRad",
        &JointController::getMotorCurrentPositionInRad, &jointController);
    ros::ServiceServer getMotorGoalPositionInRadService = n.advertiseService("GetMotorGoalPositionInRad",
        &JointController::getMotorGoalPositionInRad, &jointController);
    ros::ServiceServer setMotorGoalPositionInRadService = n.advertiseService("SetMotorGoalPositionInRad",
        &JointController::setMotorGoalPositionInRad, &jointController);
    //
    ros::ServiceServer getMotorCurrentSpeedInRadPerSecService = n.advertiseService("GetMotorCurrentSpeedInRadPerSec",
        &JointController::getMotorCurrentSpeedInRadPerSec, &jointController);
    ros::ServiceServer getMotorGoalSpeedInRadPerSecService = n.advertiseService("GetMotorGoalSpeedInRadPerSec",
        &JointController::getMotorGoalSpeedInRadPerSec, &jointController);
    ros::ServiceServer setMotorGoalSpeedInRadPerSecService = n.advertiseService("SetMotorGoalSpeedInRadPerSec",
        &JointController::setMotorGoalSpeedInRadPerSec, &jointController);
    //
    ros::ServiceServer getMotorCurrentTorqueInDecimalService = n.advertiseService("GetMotorCurrentTorqueInDecimal",
        &JointController::getMotorCurrentTorqueInDecimal, &jointController);
    ros::ServiceServer getMotorTorqueLimitInDecimalService = n.advertiseService("GetMotorTorqueLimitInDecimal",
        &JointController::getMotorTorqueLimitInDecimal, &jointController);
    ros::ServiceServer setMotorTorqueLimitInDecimalService = n.advertiseService("SetMotorTorqueLimitInDecimal",
        &JointController::setMotorTorqueLimitInDecimal, &jointController);
    //
    ros::ServiceServer getMotorCurrentPositionsInRadService = n.advertiseService("GetMotorCurrentPositionsInRad",
        &JointController::getMotorCurrentPositionsInRad, &jointController);
    ros::ServiceServer getMotorGoalPositionsInRadService = n.advertiseService("GetMotorGoalPositionsInRad",
        &JointController::getMotorGoalPositionsInRad, &jointController);
    ros::ServiceServer setMotorGoalPositionsInRadService = n.advertiseService("SetMotorGoalPositionsInRad",
        &JointController::setMotorGoalPositionsInRad, &jointController);
    //
    ros::ServiceServer getMotorCurrentSpeedsInRadPerSecService = n.advertiseService("GetMotorCurrentSpeedsInRadPerSec",
        &JointController::getMotorCurrentSpeedsInRadPerSec, &jointController);
    ros::ServiceServer getMotorGoalSpeedsInRadPerSecService = n.advertiseService("GetMotorGoalSpeedsInRadPerSec",
        &JointController::getMotorGoalSpeedsInRadPerSec, &jointController);
    ros::ServiceServer setMotorGoalSpeedsInRadPerSecService = n.advertiseService("SetMotorGoalSpeedsInRadPerSec",
        &JointController::setMotorGoalSpeedsInRadPerSec, &jointController);
    //
    ros::ServiceServer getMotorCurrentTorquesInDecimalService = n.advertiseService("GetMotorCurrentTorquesInDecimal",
        &JointController::getMotorCurrentTorquesInDecimal, &jointController);
    ros::ServiceServer getMotorTorqueLimitsInDecimalService = n.advertiseService("GetMotorTorqueLimitsInDecimal",
        &JointController::getMotorTorqueLimitsInDecimal, &jointController);
    ros::ServiceServer setMotorTorqueLimitsInDecimalService = n.advertiseService("SetMotorTorqueLimitsInDecimal",
        &JointController::setMotorTorqueLimitsInDecimal, &jointController);
    //
    ros::ServiceServer homeMotorsService = n.advertiseService("HomeAllMotors",
        &JointController::homeAllMotors, &jointController);

    // Initialise joint controller, which provides USB2AX interface and RobotHW interface for MoveIt!
    if (!jointController.init())
        return -1;

    // Initial motor settings
    usb2ax_controller::SendToAX::Request set_req;
    usb2ax_controller::SendToAX::Response set_res;
    //usb2ax_controller::ReceiveSyncFromAX::Request syncGet_req;
    //usb2ax_controller::ReceiveSyncFromAX::Response syncGet_res;
    usb2ax_controller::SetMotorParam::Request paramSet_req;
    usb2ax_controller::SetMotorParam::Response paramSet_res;
    //std_srvs::Empty::Request empty_req;
    //std_srvs::Empty::Response empty_res;

    // Reduce Return Delay Time to speed up comms
    set_req.dxlID = BROADCAST_ID;
    set_req.address = AX12_RETURN_DELAY_TIME;
    set_req.value = 0;
    jointController.sendToAX(set_req, set_res);
    ROS_INFO_STREAM("All return delay times set to " << set_req.value << ".");

    // Add compliance margins to reduce motor buzz
    set_req.dxlID = BROADCAST_ID;
    set_req.address = AX12_CW_COMPLIANCE_MARGIN;
    set_req.value = 10;
    jointController.sendToAX(set_req, set_res);
    set_req.address = AX12_CCW_COMPLIANCE_MARGIN;
    jointController.sendToAX(set_req, set_res);
    ROS_INFO_STREAM("All CW and CCW compliance margins set to " << set_req.value << ".");

//    // Set torque
//    paramSet_req.dxlID = BROADCAST_ID;
//    paramSet_req.value = 0.8;
//    jointController.setMotorTorqueLimitInDecimal(paramSet_req, paramSet_res);
//    ros::Duration(0.5).sleep();
//    ROS_INFO_STREAM("All torque limits set to " << paramSet_req.value/100.0 << "%.");

    // Set slow speed
    paramSet_req.dxlID = BROADCAST_ID;
    paramSet_req.value = 1.0;
    jointController.setMotorGoalSpeedInRadPerSec(paramSet_req, paramSet_res);
    ROS_INFO_STREAM("All goal speeds set to " << paramSet_req.value << " rad/s.";);

//    // Home all motors
//    jointController.homeAllMotors(empty_req, empty_res);
//    ros::Duration(3.0).sleep();
//    ROS_INFO("All motors homed.");

    // Turn off all torques
    set_req.dxlID = BROADCAST_ID;
    set_req.address = AX12_TORQUE_ENABLE;
    set_req.value = 0;
    jointController.sendToAX(set_req, set_res);
    ROS_INFO("All motor torques turned off.");

    // Main program loop
    ros::Time prevTime = ros::Time::now();
    while (ros::ok())
    {
        const ros::Time currentTime = ros::Time::now();

//        ROS_INFO("Current time (ms): %g", (currentTime.toNSec())/pow(10.0, 6));
//        ROS_INFO("Period (ms): %g", (currentTime - prevTime).toNSec()/pow(10.0, 6));

        jointController.read();
        jointController.cm->update(currentTime, currentTime - prevTime);
        if (jointController.getPositionControlEnabled())
            jointController.write();

        prevTime = currentTime;

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}


// C++11 init. style
const std::map<int, bool> Ax12ControlTable::addressWordMap =
{
    {AX12_MODEL_NUMBER_L, true},
    {AX12_FIRMWARE_VERSION, false},
    {AX12_ID, false},
    {AX12_BAUD_RATE, false},
    {AX12_RETURN_DELAY_TIME, false},
    {AX12_CW_ANGLE_LIMIT_L, true},
    {AX12_CCW_ANGLE_LIMIT_L, true},
    {AX12_HIGH_LIMIT_TEMPERATURE, false},
    {AX12_LOW_LIMIT_VOLTAGE, false},
    {AX12_HIGH_LIMIT_VOLTAGE, false},
    {AX12_MAX_TORQUE_L, true},
    {AX12_STATUS_RETURN_LEVEL, false},
    {AX12_ALARM_LED, false},
    {AX12_ALARM_SHUTDOWN, false},
    {AX12_TORQUE_ENABLE, false},
    {AX12_LED, false},
    {AX12_CW_COMPLIANCE_MARGIN, false},
    {AX12_CCW_COMPLIANCE_MARGIN, false},
    {AX12_CW_COMPLIANCE_SLOPE, false},
    {AX12_CCW_COMPLIANCE_SLOPE, false},
    {AX12_GOAL_POSITION_L, true},
    {AX12_MOVING_SPEED_L, true},
    {AX12_TORQUE_LIMIT_L, true},
    {AX12_PRESENT_POSITION_L, true},
    {AX12_PRESENT_SPEED_L, true},
    {AX12_PRESENT_LOAD_L, true},
    {AX12_PRESENT_VOLTAGE, false},
    {AX12_PRESENT_TEMPERATURE, false},
    {AX12_REGISTERED, false},
    {AX12_MOVING, false},
    {AX12_LOCK, false},
    {AX12_PUNCH_L, true}
};


JointController::JointController() :
    positionControlEnabled(false),
    deviceIndex(0),
    baudNum(1),
    numOfConnectedMotors(0),
    timeOfLastGoalJointStatePublication(0, 0),
    goalJointStatePublicationPeriodInMSecs(2000)
{
    connectedMotors.resize(NUM_OF_MOTORS);
    for (std::vector<bool>::iterator it = connectedMotors.begin(); it != connectedMotors.end(); ++it)
        *it = false;

    joint_state.name.resize(NUM_OF_MOTORS);
    joint_state.position.resize(NUM_OF_MOTORS);
    joint_state.velocity.resize(NUM_OF_MOTORS);
    joint_state.effort.resize(NUM_OF_MOTORS);

    directionSign.resize(NUM_OF_MOTORS);
}


JointController::~JointController()
{

}


bool JointController::init()
{
    // Note: USB2AX uses a different dxl_hal.c file than the Robotis' Dynamixel SDK for Linux.
    // The Dynamixel SDK assumes the interface is FTDI-based, and thus searches a device named ttyUSBx,
    // while the USB2AX uses the integrated CDC/ACM driver - which names the device ttyACMx.
    // The second problem is that after opening the device, the Dynamixel SDK tries to set parameters which do
    // not exist in the CDC/ACM driver.
    // For more information see:
    // http://www.xevelabs.com/doku.php?id=product:usb2ax:faq#qdynamixel_sdkhow_do_i_use_it_with_the_usb2ax

    if (positionControlEnabled)
        ROS_WARN("Position controller ENABLED. "
                 "Command values from ROS hardware_interface will be written to motors!");
    else
        ROS_WARN("Position controller DISABLED. "
                 "Command values from ROS hardware_interface will not be written to motors!");

    // Initialise comms
    if( dxl_initialize(deviceIndex, baudNum) == 0 )
    {
        ROS_ERROR("Failed to open USB2AX.");
        return false;
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
                connectedMotors[dxlID - 1] = true;
                ++numOfConnectedMotors;
                ROS_INFO("Motor with ID %d connected.", dxlID);
            }
        }

        ROS_INFO("%d motors connected.", numOfConnectedMotors);
        if (numOfConnectedMotors != NUM_OF_MOTORS)
            ROS_WARN("Number of motors should be %d.", NUM_OF_MOTORS);

        // Right arm
        joint_state.name[0] = "right_shoulder_swing_joint";
        directionSign[0] = 1;
        joint_state.name[2] = "right_shoulder_lateral_joint";
        directionSign[2] = 1;
        joint_state.name[4] = "right_elbow_joint";
        directionSign[4] = 1;

        // Left arm
        joint_state.name[1] ="left_shoulder_swing_joint";
        directionSign[1] = -1;
        joint_state.name[3] ="left_shoulder_lateral_joint";
        directionSign[3] = -1;
        joint_state.name[5] ="left_elbow_joint";
        directionSign[5] = -1;

        // Right leg
        joint_state.name[6] ="right_hip_twist_joint";
        directionSign[6] = 1;
        joint_state.name[8] ="right_hip_lateral_joint";
        directionSign[8] = -1;
        joint_state.name[10] ="right_hip_swing_joint";
        directionSign[10] = -1;
        joint_state.name[12] ="right_knee_joint";
        directionSign[12] = 1;
        joint_state.name[14] ="right_ankle_swing_joint";
        directionSign[14] = 1;
        joint_state.name[16] ="right_ankle_lateral_joint";
        directionSign[16] = 1;

        // Left leg
        joint_state.name[7] ="left_hip_twist_joint";
        directionSign[7] = -1;
        joint_state.name[9] ="left_hip_lateral_joint";
        directionSign[9] = 1;
        joint_state.name[11] ="left_hip_swing_joint";
        directionSign[11] = 1;
        joint_state.name[13] ="left_knee_joint";
        directionSign[13] = -1;
        joint_state.name[15] ="left_ankle_swing_joint";
        directionSign[15] = -1;
        joint_state.name[17] ="left_ankle_lateral_joint";
        directionSign[17] = -1;
    }

    goal_joint_state = joint_state;

    // RobotHW interface for MoveIt!
    std::vector<std::string> jointNames(NUM_OF_MOTORS);
    for (int i = 0; i < NUM_OF_MOTORS; ++i)
        jointNames[i] = joint_state.name[i];
    bioloidHw = new BioloidHw(jointNames);
    cm = new controller_manager::ControllerManager(bioloidHw);

    // Perform an initial read, and set cmd() to the initial read values, to avoid moving robot to home position
    // (at program start-up, all motors would be homed because cmd() is zero-initialised)
    read();
    for (int dxlID = 1; dxlID <= numOfConnectedMotors; ++dxlID)
        bioloidHw->setCmd( dxlID - 1, joint_state.position[dxlID - 1] );

    return true;
}


void JointController::read()
{
    const ros::Time currentTime = ros::Time::now();

    // Get position, speed and torque with a sync_read command
    joint_state.header.stamp = currentTime;
    usb2ax_controller::ReceiveSyncFromAX::Request req;
    usb2ax_controller::ReceiveSyncFromAX::Response res;
    req.dxlIDs.resize(numOfConnectedMotors);
    req.startAddress = AX12_PRESENT_POSITION_L;
    req.numOfValuesPerMotor = 3;
    for (int dxlID = 1; dxlID <= numOfConnectedMotors; ++dxlID)
    {
        if (connectedMotors[dxlID - 1])
            req.dxlIDs[dxlID - 1] = dxlID;
    }
    if ( receiveSyncFromAX(req, res) )
    {
        if ( res.values.size() < numOfConnectedMotors*3 )
            return;

        int i = 0;
        for (int dxlID = 1; dxlID <= numOfConnectedMotors; ++dxlID)
        {
            joint_state.position[dxlID - 1] = directionSign[dxlID - 1] * axPositionToRad(res.values[i++]);
            joint_state.velocity[dxlID - 1] = axSpeedToRadPerSec(res.values[i++]);
            joint_state.effort[dxlID - 1] = axTorqueToDecimal(res.values[i++]);

            bioloidHw->setPos( dxlID - 1, joint_state.position[dxlID - 1] );
            bioloidHw->setVel( dxlID - 1, joint_state.velocity[dxlID - 1] );
            bioloidHw->setEff( dxlID - 1, joint_state.effort[dxlID - 1] );
        }
    }
    jointStatePub.publish(joint_state);

    if ( ((currentTime - timeOfLastGoalJointStatePublication).toSec()*1000) >=
         goalJointStatePublicationPeriodInMSecs )
    {
        // Get goal position, goal speed and max torque with a sync_read command
        goal_joint_state.header.stamp = currentTime;
        usb2ax_controller::ReceiveSyncFromAX::Request req;
        usb2ax_controller::ReceiveSyncFromAX::Response res;
        req.dxlIDs.resize(numOfConnectedMotors);
        req.startAddress = AX12_GOAL_POSITION_L;
        req.numOfValuesPerMotor = 3;
        for (int dxlID = 1; dxlID <= numOfConnectedMotors; ++dxlID)
        {
            if (connectedMotors[dxlID - 1])
                req.dxlIDs[dxlID - 1] = dxlID;
        }
        if ( receiveSyncFromAX(req, res) )
        {
            if ( res.values.size() < numOfConnectedMotors*3 )
                return;

            int i = 0;
            for (int dxlID = 1; dxlID <= numOfConnectedMotors; ++dxlID)
            {
                goal_joint_state.position[dxlID - 1] = directionSign[dxlID - 1] * axPositionToRad(res.values[i++]);
                goal_joint_state.velocity[dxlID - 1] = axSpeedToRadPerSec(res.values[i++]);
                goal_joint_state.effort[dxlID - 1] = axTorqueToDecimal(res.values[i++]);
            }
        }
        goalJointStatePub.publish(goal_joint_state);

        timeOfLastGoalJointStatePublication = currentTime;
    }
}


void JointController::write()
{
    // Set position with a sync_write command (speed & torque not set currently)
    usb2ax_controller::SendSyncToAX::Request req;
    usb2ax_controller::SendSyncToAX::Response res;
    req.dxlIDs.resize(numOfConnectedMotors);
    req.startAddress = AX12_GOAL_POSITION_L;
    req.values.resize(numOfConnectedMotors*1);//*3);
    int i = 0;
    for (int dxlID = 1; dxlID <= numOfConnectedMotors; ++dxlID)
    {
//        ROS_INFO("Pos %d: %g", dxlID, bioloidHw->getPos(dxlID - 1));
//        ROS_INFO("Cmd %d: %g", dxlID, bioloidHw->getCmd(dxlID - 1));
        if (connectedMotors[dxlID - 1])
        {
            req.dxlIDs[dxlID - 1] = dxlID;

            req.values[i++] = radToAxPosition( directionSign[dxlID - 1] * bioloidHw->getCmd(dxlID - 1) );
//            ROS_INFO("AX value: %d", req.values[i-1]);
//            req.values[i++] = radPerSecToAxSpeed(bioloidHw->get_(dxlID - 1));
//            req.values[i++] = decimalToAxTorque(bioloidHw->get_(dxlID - 1));
        }
//        ROS_INFO("-----");
    }
//    ROS_INFO("==========");
    sendSyncToAX(req, res);
}


//void JointController::execute(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal, Server* as)
//{
//    // Do stuff
//    ROS_INFO("Executing FollowJointTrajectory!");

//    as->setSucceeded();
//}


bool JointController::receiveFromAX(usb2ax_controller::ReceiveFromAX::Request &req,
                                    usb2ax_controller::ReceiveFromAX::Response &res)
{
    // Motor
    if ( (1 <= req.dxlID) && (req.dxlID < 100) )
    {
        // Read word
        switch (req.address)
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
            res.value = dxl_read_word(req.dxlID, req.address);
            break;
        }
            // Read byte
        default:
        {
            res.value = dxl_read_byte(req.dxlID, req.address);
            break;
        }
        }
    }
    // Sensor
    else if (req.dxlID >= 100)
    {
        // Read word
        switch (req.address)
        {
        case AXS1_MODEL_NUMBER_L:
        case AXS1_SOUND_DETECTED_TIME_L:
        case AXS1_REMOCON_RX_DATA_L:
        case AXS1_REMOCON_TX_DATA_L:
        {
            res.value = dxl_read_word(req.dxlID, req.address);
            break;
        }
            // Read byte
        default:
        {
            res.value = dxl_read_byte(req.dxlID, req.address);
            break;
        }
        }
    }
    else
    {
        res.rxSuccess = false;
        return false;
    }

    int CommStatus = dxl_get_result();
    if (CommStatus == COMM_RXSUCCESS)
    {
        //ROS_DEBUG("Value received: %d", res.value);
        printErrorCode();
        res.rxSuccess = true;
        return true;
    }
    else
    {
        printCommStatus(CommStatus);
        res.rxSuccess = false;
        return false;
    }
}


bool JointController::sendToAX(usb2ax_controller::SendToAX::Request &req,
                               usb2ax_controller::SendToAX::Response &res)
{
    // Motor
    if ( ((1 <= req.dxlID) && (req.dxlID < 100)) || (req.dxlID == BROADCAST_ID) )
    {
        switch (req.address)
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
            dxl_write_word(req.dxlID, req.address, req.value);
            break;
        }
        default:
        {
            // 1 byte
            dxl_write_byte(req.dxlID, req.address, req.value);
            break;
        }
        }
    }
    // Sensor
    else if (req.dxlID >= 100)
    {
        switch (req.address)
        {
        case AXS1_MODEL_NUMBER_L:
        case AXS1_SOUND_DETECTED_TIME_L:
        case AXS1_REMOCON_RX_DATA_L:
        case AXS1_REMOCON_TX_DATA_L:
        {
            // 2 bytes
            dxl_write_word(req.dxlID, req.address, req.value);
            break;
        }
        default:
        {
            // 1 byte
            dxl_write_byte(req.dxlID, req.address, req.value);
            break;
        }
        }
    }
    else
    {
        res.txSuccess = false;
        return false;
    }

    // No return Status Packet from a broadcast command
    if (req.dxlID == BROADCAST_ID)
    {
        res.txSuccess = true;
        return true;
    }
    else
    {
        int CommStatus = dxl_get_result();
        if (CommStatus == COMM_RXSUCCESS)
        {
            //ROS_DEBUG("Value sent: %d", val);
            printErrorCode();
            res.txSuccess = true;
            return true;
        }
        else
        {
            printCommStatus(CommStatus);
            res.txSuccess = false;
            return false;
        }
    }
}


bool JointController::receiveSyncFromAX(usb2ax_controller::ReceiveSyncFromAX::Request &req,
                                        usb2ax_controller::ReceiveSyncFromAX::Response &res)
{
    // Example: IDs 1, 3, 5 - Get current position, current speed, current torque
    // startAddress:         AX12_PRESENT_POSITION_L (36)
    // numOfValuesPerMotor:  3
    // dxlIDs:               |        1      |        3      |        5      |
    // Returned values:      |  P1,  S1,  T1 |  P2,  S2,  T2 |  P3,  S3,  T3 |
    //
    // rosservice command line example:
    // rosservice call /ReceiveSyncFromAX '[1, 3, 5]' 36 3

    int numOfMotors = req.dxlIDs.size();

    if (numOfMotors <= 0)
    {
        ROS_ERROR("No motors specified.");
        res.rxSuccess = false;
        return false;
    }
    else if (numOfMotors > 32)
    {
        ROS_ERROR("Maximum number of motors must be 32.");
        res.rxSuccess = false;
        return false;
    }

    res.values.resize(req.numOfValuesPerMotor*numOfMotors);

    // Length of data for each motor
    int dataLength = 0;
    std::map<int, bool>::const_iterator it;
    std::vector<bool> isWord(req.numOfValuesPerMotor, false);
    for (int j = 0; j < req.numOfValuesPerMotor; ++j)
    {
        it = Ax12ControlTable::addressWordMap.find(req.startAddress + dataLength);
        if (it == Ax12ControlTable::addressWordMap.end())
        {
            ROS_ERROR("Address lookup error.");
            res.rxSuccess = false;
            return false;
        }
        if (it->second == true)
        {
            dataLength = dataLength + 2;
            isWord[j] = true;
        }
        else
        {
            ++dataLength;
            isWord[j] = false;
        }
    }
    if (dataLength > 6)
    {
        ROS_ERROR("Maximum data length must be 6 bytes.");
        res.rxSuccess = false;
        return false;
    }

    // Generate sync_read command
    dxl_sync_read_start(req.startAddress, dataLength);
    for (int i = 0; i < numOfMotors; ++i)
        dxl_sync_read_push_id(req.dxlIDs[i]);
    dxl_sync_read_send();

    int CommStatus = dxl_get_result();
    if (CommStatus == COMM_RXSUCCESS)
    {
        int r;
        for (int i = 0; i < numOfMotors; ++i)
        {
            for (int j = 0; j < req.numOfValuesPerMotor; ++j)
            {
                if (isWord[j])
                    r = dxl_sync_read_pop_word();
                else
                    r = dxl_sync_read_pop_byte();

//                ROS_DEBUG( "i = %d", i );
//                ROS_DEBUG( "j = %d", j );
//                ROS_DEBUG( "index = %d", i*req.numOfValuesPerMotor + j );
//                ROS_DEBUG( "ID %d = %d", req.dxlIDs[i], r );
                res.values[i*req.numOfValuesPerMotor + j] = r;
            }
        }
        printErrorCode();
        res.rxSuccess = true;
        return true;
    }
    else
    {
        printCommStatus(CommStatus);
        res.rxSuccess = false;
        return false;
    }
}


bool JointController::sendSyncToAX(usb2ax_controller::SendSyncToAX::Request &req,
                                   usb2ax_controller::SendSyncToAX::Response &res)
{
    // Example: IDs 1, 3, 5 - Set goal position 100, goal speed 300, max torque 512
    // startAddress:  AX12_GOAL_POSITION_L (30)
    // dxlIDs:        |        1      |        3      |        5      |
    // values:        | 100, 300, 512 | 100, 300, 512 | 100, 300, 512 |
    //
    // rosservice command line example:
    // rosservice call /SendSyncToAX '[1, 3, 5]' 30 '[100, 300, 512, 100, 300, 512, 100, 300, 512]'

    int numOfMotors = req.dxlIDs.size();

    if (numOfMotors <= 0)
    {
        ROS_ERROR("No motors specified.");
        res.txSuccess = false;
        return false;
    }

    int numOfValuesPerMotor = req.values.size()/numOfMotors;

    // Length of data for each motor
    int dataLength = 0;
    std::map<int, bool>::const_iterator it;
    std::vector<bool> isWord(numOfValuesPerMotor, false);
    for (int j = 0; j < numOfValuesPerMotor; ++j)
    {
        it = Ax12ControlTable::addressWordMap.find(req.startAddress + dataLength);
        if (it == Ax12ControlTable::addressWordMap.end())
        {
            ROS_ERROR("Address lookup error.");
            res.txSuccess = false;
            return false;
        }
        if (it->second == true)
        {
            dataLength = dataLength + 2;
            isWord[j] = true;
        }
        else
        {
            ++dataLength;
            isWord[j] = false;
        }
    }

    // Make sync_write packet
//    ROS_DEBUG( "Packet" );
//    ROS_DEBUG( "ID:\t\t\t %d", BROADCAST_ID );
//    ROS_DEBUG( "Instr:\t\t\t %d", INST_SYNC_WRITE );
//    ROS_DEBUG( "Param 0 (start addr):\t %d", req.startAddress );
//    ROS_DEBUG( "Param 1 (data length):\t %d", dataLength );
    dxl_set_txpacket_id(BROADCAST_ID);
    dxl_set_txpacket_instruction(INST_SYNC_WRITE);
    dxl_set_txpacket_parameter(0, req.startAddress);
    dxl_set_txpacket_parameter(1, dataLength);

    int paramIndex = 2;
    for (int i = 0; i < numOfMotors; ++i)
    {
//        ROS_DEBUG( "Param %d (dxl %d):\t %d", paramIndex, i, req.dxlIDs[i] );
        dxl_set_txpacket_parameter( paramIndex++, req.dxlIDs[i] );
        for (int j = 0; j < numOfValuesPerMotor; ++j)
        {
            //                ROS_DEBUG( "Value: %d", req.values[i*numOfValuesPerMotor + j] );
            if (isWord[j])
            {
                // 2 bytes
//                ROS_DEBUG( "Param %d (data %dL):\t %d", paramIndex, j,
//                           dxl_get_lowbyte((int)(req.values[i*numOfValuesPerMotor + j])) );
//                ROS_DEBUG( "Param %d (data %dH):\t %d", paramIndex + 1, j,
//                           dxl_get_highbyte((int)(req.values[i*numOfValuesPerMotor + j])) );
                dxl_set_txpacket_parameter( paramIndex++,
                                            dxl_get_lowbyte((int)(req.values[i*numOfValuesPerMotor + j])) );
                dxl_set_txpacket_parameter( paramIndex++,
                                            dxl_get_highbyte((int)(req.values[i*numOfValuesPerMotor + j])) );
            }
            else
            {
                // 1 byte
//                ROS_DEBUG( "Param %d (data %d):\t %d", paramIndex, j,
//                           dxl_get_highbyte((int)(req.values[i*numOfValuesPerMotor + j])) );
                dxl_set_txpacket_parameter(paramIndex++, (int)(req.values[i*numOfValuesPerMotor + j]) );
            }
//            ROS_DEBUG("--");
        }
    }

//    ROS_DEBUG( "Length:\t\t\t %d", (dataLength + 1)*numOfMotors + 4 );
    dxl_set_txpacket_length( (dataLength + 1)*numOfMotors + 4 );

    dxl_txrx_packet();

    int CommStatus = dxl_get_result();
    if (CommStatus == COMM_RXSUCCESS)
    {
        printErrorCode();
        res.txSuccess = true;
        return true;
    }
    else
    {
        printCommStatus(CommStatus);
        res.txSuccess = false;
        return false;
    }
}


bool JointController::getMotorCurrentPositionInRad(usb2ax_controller::GetMotorParam::Request &req,
                                                   usb2ax_controller::GetMotorParam::Response &res)
{
    usb2ax_controller::ReceiveFromAX::Request req2;
    usb2ax_controller::ReceiveFromAX::Response res2;
    req2.dxlID = req.dxlID;
    req2.address = AX12_PRESENT_POSITION_L;
    if ( receiveFromAX(req2, res2) )
    {
        res.value = directionSign[req.dxlID - 1] * axPositionToRad(res2.value);
        res.rxSuccess = res2.rxSuccess;
        return true;
    }
    else
    {
        res.rxSuccess = res2.rxSuccess;
        return false;
    }
}


bool JointController::getMotorGoalPositionInRad(usb2ax_controller::GetMotorParam::Request &req,
                                                usb2ax_controller::GetMotorParam::Response &res)
{
    usb2ax_controller::ReceiveFromAX::Request req2;
    usb2ax_controller::ReceiveFromAX::Response res2;
    req2.dxlID = req.dxlID;
    req2.address = AX12_GOAL_POSITION_L;
    if ( receiveFromAX(req2, res2) )
    {
        res.value = directionSign[req.dxlID - 1] * axPositionToRad(res2.value);
        res.rxSuccess = res2.rxSuccess;
        return true;
    }
    else
    {
        res.rxSuccess = res2.rxSuccess;
        return false;
    }
}


bool JointController::setMotorGoalPositionInRad(usb2ax_controller::SetMotorParam::Request &req,
                                                usb2ax_controller::SetMotorParam::Response &res)
{
    ROS_DEBUG("Direction sign: %d", directionSign[req.dxlID - 1]);
    ROS_DEBUG("Value: %d", radToAxPosition(directionSign[req.dxlID - 1] * req.value));
    ROS_DEBUG("----");
    usb2ax_controller::SendToAX::Request req2;
    usb2ax_controller::SendToAX::Response res2;
    req2.dxlID = req.dxlID;
    req2.address = AX12_GOAL_POSITION_L;
    req2.value = radToAxPosition(directionSign[req.dxlID - 1] * req.value);
    if ( sendToAX(req2, res2) )
    {
        res.txSuccess = res2.txSuccess;
        return true;
    }
    else
    {
        res.txSuccess = res2.txSuccess;
        return false;
    }
}


bool JointController::getMotorCurrentSpeedInRadPerSec(usb2ax_controller::GetMotorParam::Request &req,
                                                      usb2ax_controller::GetMotorParam::Response &res)
{
    usb2ax_controller::ReceiveFromAX::Request req2;
    usb2ax_controller::ReceiveFromAX::Response res2;
    req2.dxlID = req.dxlID;
    req2.address = AX12_PRESENT_SPEED_L;
    if ( receiveFromAX(req2, res2) )
    {
        res.value = axSpeedToRadPerSec(res2.value);
        res.rxSuccess = res2.rxSuccess;
        return true;
    }
    else
    {
        res.rxSuccess = res2.rxSuccess;
        return false;
    }
}


bool JointController::getMotorGoalSpeedInRadPerSec(usb2ax_controller::GetMotorParam::Request &req,
                                                   usb2ax_controller::GetMotorParam::Response &res)
{
    usb2ax_controller::ReceiveFromAX::Request req2;
    usb2ax_controller::ReceiveFromAX::Response res2;
    req2.dxlID = req.dxlID;
    req2.address = AX12_MOVING_SPEED_L;
    if ( receiveFromAX(req2, res2) )
    {
        res.value = axSpeedToRadPerSec(res2.value);
        res.rxSuccess = res2.rxSuccess;
        return true;
    }
    else
    {
        res.rxSuccess = res2.rxSuccess;
        return false;
    }
}


bool JointController::setMotorGoalSpeedInRadPerSec(usb2ax_controller::SetMotorParam::Request &req,
                                                   usb2ax_controller::SetMotorParam::Response &res)
{
    usb2ax_controller::SendToAX::Request req2;
    usb2ax_controller::SendToAX::Response res2;
    req2.dxlID = req.dxlID;
    req2.address = AX12_MOVING_SPEED_L;
    req2.value = radPerSecToAxSpeed(req.value);
    if ( sendToAX(req2, res2) )
    {
        res.txSuccess = res2.txSuccess;
        return true;
    }
    else
    {
        res.txSuccess = res2.txSuccess;
        return false;
    }
}


bool JointController::getMotorCurrentTorqueInDecimal(usb2ax_controller::GetMotorParam::Request &req,
                                                     usb2ax_controller::GetMotorParam::Response &res)
{
    usb2ax_controller::ReceiveFromAX::Request req2;
    usb2ax_controller::ReceiveFromAX::Response res2;
    req2.dxlID = req.dxlID;
    req2.address = AX12_PRESENT_LOAD_L;
    if ( receiveFromAX(req2, res2) )
    {
        res.value = axTorqueToDecimal(res2.value);
        res.rxSuccess = res2.rxSuccess;
        return true;
    }
    else
    {
        res.rxSuccess = res2.rxSuccess;
        return false;
    }
}


bool JointController::getMotorTorqueLimitInDecimal(usb2ax_controller::GetMotorParam::Request &req,
                                                   usb2ax_controller::GetMotorParam::Response &res)
{
    usb2ax_controller::ReceiveFromAX::Request req2;
    usb2ax_controller::ReceiveFromAX::Response res2;
    req2.dxlID = req.dxlID;
    req2.address = AX12_TORQUE_LIMIT_L;
    if ( receiveFromAX(req2, res2) )
    {
        res.value = axTorqueToDecimal(res2.value);
        res.rxSuccess = res2.rxSuccess;
        return true;
    }
    else
    {
        res.rxSuccess = res2.rxSuccess;
        return false;
    }
}


bool JointController::setMotorTorqueLimitInDecimal(usb2ax_controller::SetMotorParam::Request &req,
                                                   usb2ax_controller::SetMotorParam::Response &res)
{
    usb2ax_controller::SendToAX::Request req2;
    usb2ax_controller::SendToAX::Response res2;
    req2.dxlID = req.dxlID;
    req2.address = AX12_TORQUE_LIMIT_L;
    req2.value = decimalToAxTorque(req.value);
    if ( sendToAX(req2, res2) )
    {
        res.txSuccess = res2.txSuccess;
        return true;
    }
    else
    {
        res.txSuccess = res2.txSuccess;
        return false;
    }
}


bool JointController::getMotorCurrentPositionsInRad(usb2ax_controller::GetMotorParams::Request &req,
                                                    usb2ax_controller::GetMotorParams::Response &res)
{
    usb2ax_controller::ReceiveSyncFromAX::Request req2;
    usb2ax_controller::ReceiveSyncFromAX::Response res2;
    req2.dxlIDs = req.dxlIDs;
    req2.startAddress = AX12_PRESENT_POSITION_L;
    req2.numOfValuesPerMotor = 1;
    if ( receiveSyncFromAX(req2, res2) )
    {
        res.values.resize(res2.values.size());
        for (int i = 0; i < req2.dxlIDs.size(); ++i)
            res.values[i] = directionSign[req2.dxlIDs[i] - 1] * axPositionToRad(res2.values[i]);
        res.rxSuccess = res2.rxSuccess;
        return true;
    }
    else
    {
        res.rxSuccess = res2.rxSuccess;
        return false;
    }
}


bool JointController::getMotorGoalPositionsInRad(usb2ax_controller::GetMotorParams::Request &req,
                                                 usb2ax_controller::GetMotorParams::Response &res)
{
    usb2ax_controller::ReceiveSyncFromAX::Request req2;
    usb2ax_controller::ReceiveSyncFromAX::Response res2;
    req2.dxlIDs = req.dxlIDs;
    req2.startAddress = AX12_GOAL_POSITION_L;
    req2.numOfValuesPerMotor = 1;
    if ( receiveSyncFromAX(req2, res2) )
    {
        res.values.resize(res2.values.size());
        for (int i = 0; i < req2.dxlIDs.size(); ++i)
            res.values[i] = directionSign[req2.dxlIDs[i] - 1] * axPositionToRad(res2.values[i]);
        res.rxSuccess = res2.rxSuccess;
        return true;
    }
    else
    {
        res.rxSuccess = res2.rxSuccess;
        return false;
    }
}


bool JointController::setMotorGoalPositionsInRad(usb2ax_controller::SetMotorParams::Request &req,
                                                 usb2ax_controller::SetMotorParams::Response &res)
{
    if ( req.dxlIDs.size() != req.values.size() )
    {
        ROS_ERROR("Input data size mismatch.");
        return false;
    }

    usb2ax_controller::SendSyncToAX::Request req2;
    usb2ax_controller::SendSyncToAX::Response res2;
    req2.dxlIDs = req.dxlIDs;
    req2.startAddress = AX12_GOAL_POSITION_L;
    req2.values.resize(req.values.size());
    for (int i = 0; i < req2.dxlIDs.size(); ++i)
        req2.values[i] = radToAxPosition( directionSign[req2.dxlIDs[i] - 1] * req.values[i] );
    if ( sendSyncToAX(req2, res2) )
        return true;
    else
        return false;
}


bool JointController::getMotorCurrentSpeedsInRadPerSec(usb2ax_controller::GetMotorParams::Request &req,
                                                       usb2ax_controller::GetMotorParams::Response &res)
{
    usb2ax_controller::ReceiveSyncFromAX::Request req2;
    usb2ax_controller::ReceiveSyncFromAX::Response res2;
    req2.dxlIDs = req.dxlIDs;
    req2.startAddress = AX12_PRESENT_SPEED_L;
    req2.numOfValuesPerMotor = 1;
    if ( receiveSyncFromAX(req2, res2) )
    {
        res.values.resize(res2.values.size());
        for (int i = 0; i < req2.dxlIDs.size(); ++i)
            res.values[i] = axSpeedToRadPerSec(res2.values[i]);
        res.rxSuccess = res2.rxSuccess;
        return true;
    }
    else
    {
        res.rxSuccess = res2.rxSuccess;
        return false;
    }
}


bool JointController::getMotorGoalSpeedsInRadPerSec(usb2ax_controller::GetMotorParams::Request &req,
                                                    usb2ax_controller::GetMotorParams::Response &res)
{
    usb2ax_controller::ReceiveSyncFromAX::Request req2;
    usb2ax_controller::ReceiveSyncFromAX::Response res2;
    req2.dxlIDs = req.dxlIDs;
    req2.startAddress = AX12_MOVING_SPEED_L;
    req2.numOfValuesPerMotor = 1;
    if ( receiveSyncFromAX(req2, res2) )
    {
        res.values.resize(res2.values.size());
        for (int i = 0; i < req2.dxlIDs.size(); ++i)
            res.values[i] = axSpeedToRadPerSec(res2.values[i]);
        res.rxSuccess = res2.rxSuccess;
        return true;
    }
    else
    {
        res.rxSuccess = res2.rxSuccess;
        return false;
    }
}


bool JointController::setMotorGoalSpeedsInRadPerSec(usb2ax_controller::SetMotorParams::Request &req,
                                                    usb2ax_controller::SetMotorParams::Response &res)
{
    if ( req.dxlIDs.size() != req.values.size() )
    {
        ROS_ERROR("Input data size mismatch.");
        return false;
    }

    usb2ax_controller::SendSyncToAX::Request req2;
    usb2ax_controller::SendSyncToAX::Response res2;
    req2.dxlIDs = req.dxlIDs;
    req2.startAddress = AX12_MOVING_SPEED_L;
    req2.values.resize(req.values.size());
    for (int i = 0; i < req2.dxlIDs.size(); ++i)
        req2.values[i] = radPerSecToAxSpeed(req.values[i]);
    if ( sendSyncToAX(req2, res2) )
        return true;
    else
        return false;
}


bool JointController::getMotorCurrentTorquesInDecimal(usb2ax_controller::GetMotorParams::Request &req,
                                                      usb2ax_controller::GetMotorParams::Response &res)
{
    usb2ax_controller::ReceiveSyncFromAX::Request req2;
    usb2ax_controller::ReceiveSyncFromAX::Response res2;
    req2.dxlIDs = req.dxlIDs;
    req2.startAddress = AX12_PRESENT_LOAD_L;
    req2.numOfValuesPerMotor = 1;
    if ( receiveSyncFromAX(req2, res2) )
    {
        res.values.resize(res2.values.size());
        for (int i = 0; i < req2.dxlIDs.size(); ++i)
            res.values[i] = axTorqueToDecimal(res2.values[i]);
        res.rxSuccess = res2.rxSuccess;
        return true;
    }
    else
    {
        res.rxSuccess = res2.rxSuccess;
        return false;
    }
}


bool JointController::getMotorTorqueLimitsInDecimal(usb2ax_controller::GetMotorParams::Request &req,
                                                    usb2ax_controller::GetMotorParams::Response &res)
{
    usb2ax_controller::ReceiveSyncFromAX::Request req2;
    usb2ax_controller::ReceiveSyncFromAX::Response res2;
    req2.dxlIDs = req.dxlIDs;
    req2.startAddress = AX12_MAX_TORQUE_L;
    req2.numOfValuesPerMotor = 1;
    if ( receiveSyncFromAX(req2, res2) )
    {
        res.values.resize(res2.values.size());
        for (int i = 0; i < req2.dxlIDs.size(); ++i)
            res.values[i] = axTorqueToDecimal(res2.values[i]);
        res.rxSuccess = res2.rxSuccess;
        return true;
    }
    else
    {
        res.rxSuccess = res2.rxSuccess;
        return false;
    }
}


bool JointController::setMotorTorqueLimitsInDecimal(usb2ax_controller::SetMotorParams::Request &req,
                                                    usb2ax_controller::SetMotorParams::Response &res)
{
    if ( req.dxlIDs.size() != req.values.size() )
    {
        ROS_ERROR("Input data size mismatch.");
        return false;
    }

    usb2ax_controller::SendSyncToAX::Request req2;
    usb2ax_controller::SendSyncToAX::Response res2;
    req2.dxlIDs = req.dxlIDs;
    req2.startAddress = AX12_MAX_TORQUE_L;
    req2.values.resize(req.values.size());
    for (int i = 0; i < req2.dxlIDs.size(); ++i)
        req2.values[i] = decimalToAxTorque(req.values[i]);
    if ( sendSyncToAX(req2, res2) )
        return true;
    else
        return false;
}


bool JointController::homeAllMotors(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
//    usb2ax_controller::SendSyncToAX::Request req2;
//    usb2ax_controller::SendSyncToAX::Response res2;
//    req2.dxlIDs.resize(numOfConnectedMotors);
//    req2.startAddress = AX12_GOAL_POSITION_L;
//    req2.values.resize(numOfConnectedMotors*1);
//    for (int dxlID = 1; dxlID <= numOfConnectedMotors; ++dxlID)
//    {
//        if (connectedMotors[dxlID - 1])
//        {
//            req2.dxlIDs[dxlID - 1] = dxlID;
//            req2.values[dxlID - 1] = radToAxPosition(0.0);
//        }
//    }
//    if ( sendSyncToAX(req2, res2) )
//        return true;
//    else
//        return false;

    // Straightforward way: Use BROADCAST_ID
    usb2ax_controller::SendToAX::Request req2;
    usb2ax_controller::SendToAX::Response res2;
    req2.dxlID = BROADCAST_ID;
    req2.address = AX12_GOAL_POSITION_L;
    req2.value = radToAxPosition(0.0);
    if ( sendToAX(req2, res2) )
        return true;
    else
        return false;
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
    // 0.29 degrees per unit <=> ~0.005 rads per unit
    // Position range: 0..1023 <=> 0..296.67 degrees <=> 0..5.115 rad
    // If offset is added so that 0 is the midpoint (0 degrees), then range is:
    // -512..511 <=> -148.48..148.19 degrees <=> -2.56..2.555 rad
    return ((oldValue & 0x3FF) - 512)*0.005;  // Bits 0-9
}


int JointController::radToAxPosition(float oldValue)
{
    // Convert rads to AX-12 position
    if ( ((-512*0.005 - FLOAT_PRECISION_THRESH) <= oldValue) and
         (oldValue <= (511*0.005 + FLOAT_PRECISION_THRESH)) )
    {
        return round( oldValue/0.005 + 512 );
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
    // v (Hz) = w (rad/s) / 2*pi
    // 1 rpm = 1/60 Hz ~= 0.105 rad/s
    // ~0.111 rpm per unit <=> ~0.012 rad/s per unit
    // Speed range:    0..1023 <=> 0..113.553 rpm (CCW) <=> 0..12.276 rad/s (CCW)
    //              1024..2047 <=> 0..-113.553 rpm (CW)  <=> 0..-12.276 rad/s (CW)
    float newValue = (oldValue & 0x3FF)*0.012;  // Bits 0-9
    if ( (oldValue & 0x400) == 0x0 )  // Check bit 10
        return newValue;
    else
        return -newValue;
}


int JointController::radPerSecToAxSpeed(float oldValue)
{
    // Convert rads per sec to AX-12 speed
    int newValue = round( fabs(oldValue)/0.012 );
    if ( (0.0 <= oldValue) && (oldValue <= (1023*0.012 + FLOAT_PRECISION_THRESH)) )
        return newValue;
    else if ( ((-1023*0.012 - FLOAT_PRECISION_THRESH) <= oldValue) && (oldValue < 0.0) )
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
    // Torque range:    0..1023 <=> 0.0..1.023 (CCW)
    //               1024..2047 <=> 0.0..-1.023 (CW)
    float newValue = (oldValue & 0x3FF)*0.001;  // Bits 0-9
    if ( (oldValue & 0x400) == 0x0 )  // Check bit 10
        return newValue;
    else
        return -newValue;
}


int JointController::decimalToAxTorque(float oldValue)
{
    // Convert % torque to AX-12 torque
    int newValue = round( fabs(oldValue)/0.001 );
    if ( (0.0 <= oldValue) && (oldValue <= (1023*0.001 + FLOAT_PRECISION_THRESH)) )
        return newValue;
    else if ( ((-1023*0.001 - FLOAT_PRECISION_THRESH) <= oldValue) && (oldValue < 0.0) )
        return newValue | 0x400;  // Set bit 10 to 1
    else
    {
        ROS_WARN("Value outside of valid input range, returning 0.");
        return 0;
    }
}
