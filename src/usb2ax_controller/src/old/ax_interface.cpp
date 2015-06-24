
#include "ax_interface.h"
#include "ros/ros.h"
#include "usb2ax/dynamixel.h"
#include "ax12ControlTableMacros.h"
#include "axs1ControlTableMacros.h"
#include "usb2ax_controller/AX12State.h"
#include "usb2ax_controller/AXS1State.h"
#include <sstream>

// Note: USB2AX uses a different dxl_hal.c file than the Robotis' Dynamixel SDK for Linux.
// The Dynamixel SDK assumes the interface is FTDI-based, and thus searches a device named ttyUSBx,
// while the USB2AX uses the integrated CDC/ACM driver - which names the device ttyACMx.
// The second problem is that after opening the device, the Dynamixel SDK tries to set parameters which do
// not exist in the CDC/ACM driver.
// For more information see:
// http://www.xevelabs.com/doku.php?id=product:usb2ax:faq#qdynamixel_sdkhow_do_i_use_it_with_the_usb2ax

#define NUM_OF_MOTORS 18
#define NUM_OF_SENSORS 1

// IDs 1-99 are assumed used for motors
// IDs 100 and above are assumed used for sensors


int main(int argc, char **argv)
{
    // Argument 1: Device index, default = 0
    // Argument 2: Baud number, default = 1
    int deviceIndex = 0;
    int baudNum = 1;
    if (argc >= 2)
    {
        int val;
        std::stringstream iss(argv[1]);
        if (iss >> val)
            deviceIndex = val;
    }
    if (argc >= 3)
    {
        int val;
        std::istringstream iss(argv[2]);
        if (iss >> val)
            baudNum = val;
    }

    // Initialise comms
    if( dxl_initialize(deviceIndex, baudNum) == 0 )
        ROS_ERROR("Failed to open USB2AX.");
    else
        ROS_INFO("USB2AX opened successfully.");

    // Setup ROS
    ros::init(argc, argv, "ax_interface");
    ros::NodeHandle n;
    ros::Rate loop_rate(1000);  // Hz

    // Find motors with IDs 1-NUM_OF_MOTORS
    bool connectedMotors[NUM_OF_MOTORS] = {false};
    ros::Publisher motorStatePub[NUM_OF_MOTORS];
    for (int dxlID = 1; dxlID <= NUM_OF_MOTORS; ++dxlID)
    {
        dxl_ping(dxlID);
        if(dxl_get_result() == COMM_RXSUCCESS)
        {
            connectedMotors[dxlID-1] = true;
            ROS_INFO("Motor with ID %d connected.", dxlID);

            // State message publisher
            std::stringstream ss;
            ss << dxlID;
            std::string topicName = "State/Motor";
            topicName += ss.str();
            motorStatePub[dxlID-1] = n.advertise<usb2ax_controller::AX12State>(topicName.c_str(), 1000);
        }
    }

    // Find sensors with IDs 1-NUM_OF_SENSORS
    bool connectedSensors[NUM_OF_SENSORS] = {false};
    ros::Publisher sensorStatePub[NUM_OF_SENSORS];
    for (int dxlID = 100; dxlID <= 100 + NUM_OF_SENSORS; ++dxlID)
    {
        dxl_ping(dxlID);
        if(dxl_get_result() == COMM_RXSUCCESS)
        {
            connectedSensors[dxlID-100] = true;
            ROS_INFO("Sensor with ID %d connected.", dxlID);

            // State message publisher
            std::stringstream ss;
            ss << dxlID;
            std::string topicName = "State/Sensor";
            topicName += ss.str();
            sensorStatePub[dxlID-100] = n.advertise<usb2ax_controller::AXS1State>(topicName.c_str(), 1000);
        }
    }

    // Send service
    ros::ServiceServer sendtoAXService = n.advertiseService("SendToAX", sendToAX);

    while (ros::ok())
    {

        //Get state feedback for each connected motor
        for (int dxlID = 1; dxlID <= NUM_OF_MOTORS; ++dxlID)
        {
            if (connectedMotors[dxlID-1])
            {
                bool messageSuccess = true;
                int tmpVal;
                usb2ax_controller::AX12State msg;

                if (getValue(dxlID, AX12_PRESENT_POSITION_L, tmpVal) == 0)
                    msg.position = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AX12_PRESENT_SPEED_L, tmpVal) == 0)
                    msg.speed = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AX12_PRESENT_LOAD_L, tmpVal) == 0)
                    msg.load = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AX12_PRESENT_VOLTAGE, tmpVal) == 0)
                    msg.voltage = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AX12_PRESENT_TEMPERATURE, tmpVal) == 0)
                    msg.temperature = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AX12_MOVING, tmpVal) == 0)
                    msg.isMoving = tmpVal;
                else
                    messageSuccess = false;

                if (messageSuccess)
                    motorStatePub[dxlID-1].publish(msg);
            }
        }

        //Get state feedback for each connected sensor
        for (int dxlID = 100; dxlID < (100 + NUM_OF_SENSORS); ++dxlID)
        {
            if (connectedSensors[dxlID-100])
            {
                bool messageSuccess = true;
                int tmpVal;
                usb2ax_controller::AXS1State msg;

                if (getValue(dxlID, AXS1_IR_LEFT_FIRE_DATA, tmpVal) == 0)
                    msg.irLeft = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AXS1_IR_CENTRE_FIRE_DATA, tmpVal) == 0)
                    msg.irCentre = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AXS1_IR_RIGHT_FIRE_DATA, tmpVal) == 0)
                    msg.irRight = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AXS1_LIGHT_LEFT_DATA, tmpVal) == 0)
                    msg.lightLeft = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AXS1_LIGHT_CENTRE_DATA, tmpVal) == 0)
                    msg.lightCentre = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AXS1_LIGHT_RIGHT_DATA, tmpVal) == 0)
                    msg.lightRight = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AXS1_IR_OBSTACLE_DETECTED, tmpVal) == 0)
                    msg.irObstacleDetected = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AXS1_LIGHT_DETECTED, tmpVal) == 0)
                    msg.isLightDetected = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AXS1_SOUND_DATA, tmpVal) == 0)
                    msg.sound = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AXS1_SOUND_DETECTED_COUNT, tmpVal) == 0)
                    msg.soundDetectedCount = tmpVal;
                else
                    messageSuccess = false;
                if (getValue(dxlID, AXS1_SOUND_DETECTED_TIME_L, tmpVal) == 0)
                    msg.soundDetectedTime = tmpVal;
                else
                    messageSuccess = false;

                if (messageSuccess)
                    sensorStatePub[dxlID-100].publish(msg);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}


bool getValue(int dxlID, int controlTableAddr, int& val)
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

    int CommStatus = dxl_get_result();
    if (CommStatus == COMM_RXSUCCESS)
    {
        //ROS_INFO("Value received: %d", val);
        printErrorCode();
        return 0;
    }
    else
    {
        printCommStatus(CommStatus);
        return -1;
    }
}


bool setValue(int dxlID, int controlTableAddr, int val)
{
    // Motor
    if ( (1 <= dxlID) and (dxlID < 100) )
    {
        // Write word
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
            dxl_write_word(dxlID, controlTableAddr, val);
            break;
        }
        // Write byte
        default:
        {
            dxl_write_byte(dxlID, controlTableAddr, val);
            break;
        }
        }
    }
    // Sensor
    else if (dxlID >= 100)
    {
        // Write word
        switch (controlTableAddr)
        {
        case AXS1_MODEL_NUMBER_L:
        case AXS1_SOUND_DETECTED_TIME_L:
        case AXS1_REMOCON_RX_DATA_L:
        case AXS1_REMOCON_TX_DATA_L:
        {
            dxl_write_word(dxlID, controlTableAddr, val);
            break;
        }
        // Write byte
        default:
        {
            dxl_write_byte(dxlID, controlTableAddr, val);
            break;
        }
        }
    }

    int CommStatus = dxl_get_result();
    if (CommStatus == COMM_TXSUCCESS)
    {
        ROS_INFO("Value sent: %d", val);
        printErrorCode();
        return 0;
    }
    else
    {
        printCommStatus(CommStatus);
        return -1;
    }
}


void printCommStatus(int CommStatus)
{
    switch(CommStatus)
    {
    case COMM_TXFAIL:
    {
        ROS_ERROR("COMM_TXFAIL: Failed transmit instruction packet!");
        break;
    }
    case COMM_TXERROR:
    {
        ROS_ERROR("COMM_TXERROR: Incorrect instruction packet!");
        break;
    }
    case COMM_RXFAIL:
    {
        ROS_ERROR("COMM_RXFAIL: Failed get status packet from device!");
        break;
    }
    case COMM_RXWAITING:
    {
        ROS_ERROR("COMM_RXWAITING: Now recieving status packet!");
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


void printErrorCode()
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


bool sendToAX(usb2ax_controller::SendToAX::Request &req, usb2ax_controller::SendToAX::Response &res)
{
    // Send value
    res.txSuccess = setValue(req.dxlID, req.address, req.value);


//    ROS_INFO("Request: ID: %d, Goal: %d", req.dxlID, req.goalPosition);

//    dxl_write_word( req.dxlID, AX12_GOAL_POSITION_L, req.goalPosition );

//    // Check moving
//    int Moving = dxl_read_byte( req.dxlID, AX12_MOVING );
//    int CommStatus = dxl_get_result();
//    if( CommStatus == COMM_RXSUCCESS )
//    {
//        res.isMoving = Moving;
//    }
//    else
//    {
//        res.isMoving = 0;
//        printCommStatus(CommStatus);
//    }
//    ROS_INFO("Sending back response: %d", res.isMoving);

    return true;
}

