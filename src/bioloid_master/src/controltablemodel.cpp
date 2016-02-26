#include "controltablemodel.h"
#include "../../usb2ax_controller/src/ax12ControlTableMacros.h"


ControlTableModel::ControlTableModel(QObject *parent) :
    QAbstractTableModel(parent)
{
    initControlTableRows();
}


int ControlTableModel::rowCount(const QModelIndex &parent) const
{
   return controlTableRows.size();
}


int ControlTableModel::columnCount(const QModelIndex &parent) const
{
    return 5;
}


QVariant ControlTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::DisplayRole)
    {
        if (orientation == Qt::Horizontal)
        {
            switch (section)
            {
            case 0:
                return QString("Area");
//            case 1:
//                return QString("Address");
            case 1:
                return QString("Name");
            case 2:
                return QString("Description");
            case 3:
                return QString("Write Access");
            case 4:
                return QString("Value");
//            case 6:
//                return QString("Write Value");
            default:
                break;
            }
        }
//        else if (orientation == Qt::Vertical)
//            return QString::number(section);
    }
    return QVariant();
}


QVariant ControlTableModel::data(const QModelIndex &index, int role) const
{
    if (role == Qt::DisplayRole)
    {
        int r = index.row();
        int c = index.column();
        switch (c)
        {
        case 0:
            return controlTableRows[r]->area;
//        case 1:
//            return QString::number(controlTableRows[r]->address);
        case 1:
            return controlTableRows[r]->name;
        case 2:
            return controlTableRows[r]->description;
        case 3:
            if (controlTableRows[r]->writeAccess)
                return "True";
            else
                return "False";
        case 4:
            return QString::number(controlTableRows[r]->value);
//        case 6:
//            return QString::number(controlTableRows[r]->targetValue);
        default:
            break;
        }
    }
    return QVariant();
}


bool ControlTableModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if (role == Qt::EditRole)
    {
        int r = index.row();
        int c = index.column();
        switch (c)
        {
        case 4:
            controlTableRows[r]->value = value.toUInt();
            emit dataChanged(index, index);
            break;
//        case 6:
//            controlTableRows[r]->targetValue = value.toUInt();
//            emit dataChanged(index, index);
//            break;
        default:
            break;
        }
    }
    else if (role == Qt::UserRole)
    {
        int r = index.row();
        int c = index.column();
        switch (c)
        {
        case 4:
            controlTableRows[r]->value = value.toUInt();
            emit dataChanged(index, index);
            emit sendData(controlTableRows[r]->name, controlTableRows[r]->address, controlTableRows[r]->value);
            break;
//        case 6:
//            controlTableRows[r]->targetValue = value.toUInt();
//            break;
        default:
            break;
        }
    }
    return true;
}


Qt::ItemFlags ControlTableModel::flags(const QModelIndex &index) const
{
    int r = index.row();
    int c = index.column();
    switch (c)
    {
    case 4:
        if(controlTableRows[r]->writeAccess)
            return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable;
        else
            return Qt::ItemIsSelectable;
    default:
        break;
    }
    return QAbstractTableModel::flags(index);
}


void ControlTableModel::initControlTableRows()
{
    int numOfRows = 32;//43;
    //int numOfEepromRows = 14;//18;
    //int numOfRamRows = 18;//25;

    controlTableRows.resize(numOfRows);
    for (int r = 0; r < numOfRows; ++r)
        controlTableRows[r] = new ControlTableRow;

    int r = 0;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_MODEL_NUMBER_L;
    //controlTableRows[r]->name = "Model Number (L)";
    controlTableRows[r]->name = "Model Number";
    //controlTableRows[r]->description = "Lowest byte of model number";
    controlTableRows[r]->description = "Model number";
    controlTableRows[r]->writeAccess = false;

//    ++r;
//    controlTableRows[r]->area = "EEPROM";
//    controlTableRows[r]->address = AX12_MODEL_NUMBER_H;
//    controlTableRows[r]->name = "Model Number (H)";
//    controlTableRows[r]->description = "Highest byte of model number";
//    controlTableRows[r]->writeAccess = false;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_FIRMWARE_VERSION;
    controlTableRows[r]->name = "Version of Firmware";
    controlTableRows[r]->description = "Information on the version of firmware";
    controlTableRows[r]->writeAccess = false;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_ID;
    controlTableRows[r]->name = "ID";
    controlTableRows[r]->description = "ID of Dynamixel";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_BAUD_RATE;
    controlTableRows[r]->name = "Baud Rate";
    controlTableRows[r]->description = "Baud Rate of Dynamixel";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_RETURN_DELAY_TIME;
    controlTableRows[r]->name = "Return Delay Time";
    controlTableRows[r]->description = "Return Delay Time";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_CW_ANGLE_LIMIT_L;
    controlTableRows[r]->name = "CW Angle Limit";// (L)";
    //controlTableRows[r]->description = "Lowest byte of Clockwise Angle Limit";
    controlTableRows[r]->description = "Clockwise Angle Limit";
    controlTableRows[r]->writeAccess = true;

//    ++r;
//    controlTableRows[r]->area = "EEPROM";
//    controlTableRows[r]->address = AX12_CW_ANGLE_LIMIT_H;
//    controlTableRows[r]->name = "CW Angle Limit (L)";
//    controlTableRows[r]->description = "Highest byte of Clockwise Angle Limit";
//    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_CCW_ANGLE_LIMIT_L;
    controlTableRows[r]->name = "CCW Angle Limit";// (L)";
    //controlTableRows[r]->description = "Lowest byte of Counter-Clockwise Angle Limit";
    controlTableRows[r]->description = "Counter-Clockwise Angle Limit";
    controlTableRows[r]->writeAccess = true;

//    ++r;
//    controlTableRows[r]->area = "EEPROM";
//    controlTableRows[r]->address = AX12_CCW_ANGLE_LIMIT_H;
//    controlTableRows[r]->name = "CCW Angle Limit (H)";
//    controlTableRows[r]->description = "Highest byte of Counter-Clockwise Angle Limit";
//    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_HIGH_LIMIT_TEMPERATURE;
    controlTableRows[r]->name = "Highest Limit Temperature";
    controlTableRows[r]->description = "Internal Limit Temperature";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_LOW_LIMIT_VOLTAGE;
    controlTableRows[r]->name = "Lowest Limit Voltage";
    controlTableRows[r]->description = "Lowest Limit Voltage";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_HIGH_LIMIT_VOLTAGE;
    controlTableRows[r]->name = "Highest Limit Voltage";
    controlTableRows[r]->description = "Highest Limit Voltage";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_MAX_TORQUE_L;
    controlTableRows[r]->name = "Max Torque";// (L)";
    //controlTableRows[r]->description = "Lowest byte of Maximum Torque";
    controlTableRows[r]->description = "Maximum Torque";
    controlTableRows[r]->writeAccess = true;

//    ++r;
//    controlTableRows[r]->area = "EEPROM";
//    controlTableRows[r]->address = AX12_MAX_TORQUE_H;
//    controlTableRows[r]->name = "Max Torque (H)";
//    controlTableRows[r]->description = "Highest byte of Maximum Torque";
//    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_STATUS_RETURN_LEVEL;
    controlTableRows[r]->name = "Status Return Level";
    controlTableRows[r]->description = "Status Return Level";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_ALARM_LED;
    controlTableRows[r]->name = "Alarm LED";
    controlTableRows[r]->description = "LED for Alarm";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "EEPROM";
    controlTableRows[r]->address = AX12_ALARM_SHUTDOWN;
    controlTableRows[r]->name = "Alarm Shutdown";
    controlTableRows[r]->description = "Shutdown for Alarm";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_TORQUE_ENABLE;
    controlTableRows[r]->name = "Torque Enable";
    controlTableRows[r]->description = "Torque On/Off";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_LED;
    controlTableRows[r]->name = "LED";
    controlTableRows[r]->description = "LED On/Off";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_CW_COMPLIANCE_MARGIN;
    controlTableRows[r]->name = "CW Compliance Margin";
    controlTableRows[r]->description = "Clockwise Compliance Margin";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_CCW_COMPLIANCE_MARGIN;
    controlTableRows[r]->name = "CCW Compliance Margin";
    controlTableRows[r]->description = "Counter-Clockwise Compliance Margin";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_CW_COMPLIANCE_SLOPE;
    controlTableRows[r]->name = "CW Compliance Slope";
    controlTableRows[r]->description = "Clockwise Compliance Slope";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_CCW_COMPLIANCE_SLOPE;
    controlTableRows[r]->name = "CCW Compliance Slope";
    controlTableRows[r]->description = "Counter-Clockwise Compliance Slope";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_GOAL_POSITION_L;
    controlTableRows[r]->name = "Goal Position";// (L)";
    //controlTableRows[r]->description = "Lowest byte of Goal Position";
    controlTableRows[r]->description = "Goal Position";
    controlTableRows[r]->writeAccess = true;

//    ++r;
//    controlTableRows[r]->area = "RAM";
//    controlTableRows[r]->address = AX12_GOAL_POSITION_H;
//    controlTableRows[r]->name = "Goal Position (H)";
//    controlTableRows[r]->description = "Highest byte of Goal Position";
//    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_MOVING_SPEED_L;
    controlTableRows[r]->name = "Moving Speed";// (L)";
    //controlTableRows[r]->description = "Lowest byte of Moving Speed (Moving Velocity)";
    controlTableRows[r]->description = "Moving Speed (Moving Velocity)";
    controlTableRows[r]->writeAccess = true;

//    ++r;
//    controlTableRows[r]->area = "RAM";
//    controlTableRows[r]->address = AX12_MOVING_SPEED_H;
//    controlTableRows[r]->name = "Moving Speed (H)";
//    controlTableRows[r]->description = "Highest byte of Moving Speed (Moving Velocity)";
//    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_TORQUE_LIMIT_L;
    controlTableRows[r]->name = "Torque Limit";// (L)";
    //controlTableRows[r]->description = "Lowest byte of Torque Limit (Goal Torque)";
    controlTableRows[r]->description = "Torque Limit (Goal Torque)";
    controlTableRows[r]->writeAccess = true;

//    ++r;
//    controlTableRows[r]->area = "RAM";
//    controlTableRows[r]->address = AX12_TORQUE_LIMIT_H;
//    controlTableRows[r]->name = "Torque Limit (H)";
//    controlTableRows[r]->description = "Highest byte of Torque Limit (Goal Torque)";
//    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_PRESENT_POSITION_L;
    controlTableRows[r]->name = "Present Position";// (L)";
    //controlTableRows[r]->description = "Lowest byte of Current Position";
    controlTableRows[r]->description = "Current Position";
    controlTableRows[r]->writeAccess = false;

//    ++r;
//    controlTableRows[r]->area = "RAM";
//    controlTableRows[r]->address = AX12_PRESENT_POSITION_H;
//    controlTableRows[r]->name = "Present Position (H)";
//    controlTableRows[r]->description = "Highest byte of Current Position";
//    controlTableRows[r]->writeAccess = false;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_PRESENT_SPEED_L;
    controlTableRows[r]->name = "Present Speed";// (L)";
    //controlTableRows[r]->description = "Lowest byte of Current Speed";
    controlTableRows[r]->description = "Current Speed";
    controlTableRows[r]->writeAccess = false;

//    ++r;
//    controlTableRows[r]->area = "RAM";
//    controlTableRows[r]->address = AX12_PRESENT_SPEED_H;
//    controlTableRows[r]->name = "Present Speed (H)";
//    controlTableRows[r]->description = "Highest byte of Current Speed";
//    controlTableRows[r]->writeAccess = false;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_PRESENT_LOAD_L;
    controlTableRows[r]->name = "Present Load";// (L)";
    //controlTableRows[r]->description = "Lowest byte of Current Load";
    controlTableRows[r]->description = "Current Load";
    controlTableRows[r]->writeAccess = false;

//    ++r;
//    controlTableRows[r]->area = "RAM";
//    controlTableRows[r]->address = AX12_PRESENT_LOAD_H;
//    controlTableRows[r]->name = "Present Load (H)";
//    controlTableRows[r]->description = "Highest byte of Current Load";
//    controlTableRows[r]->writeAccess = false;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_PRESENT_VOLTAGE;
    controlTableRows[r]->name = "Present Voltage";
    controlTableRows[r]->description = "Current Voltage";
    controlTableRows[r]->writeAccess = false;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_PRESENT_TEMPERATURE;
    controlTableRows[r]->name = "Present Temperature";
    controlTableRows[r]->description = "Current Temperature";
    controlTableRows[r]->writeAccess = false;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_REGISTERED;
    controlTableRows[r]->name = "Registered";
    controlTableRows[r]->description = "Means if instruction is registered";
    controlTableRows[r]->writeAccess = false;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_MOVING;
    controlTableRows[r]->name = "Moving";
    controlTableRows[r]->description = "Means if there is any movement";
    controlTableRows[r]->writeAccess = false;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_LOCK;
    controlTableRows[r]->name = "Lock";
    controlTableRows[r]->description = "Locking EEPROM";
    controlTableRows[r]->writeAccess = true;

    ++r;
    controlTableRows[r]->area = "RAM";
    controlTableRows[r]->address = AX12_PUNCH_L;
    controlTableRows[r]->name = "Punch";// (L)";
    //controlTableRows[r]->description = "Lowest byte of Punch";
    controlTableRows[r]->description = "Punch";
    controlTableRows[r]->writeAccess = true;

//    ++r;
//    controlTableRows[r]->area = "RAM";
//    controlTableRows[r]->address = AX12_PUNCH_H;
//    controlTableRows[r]->name = "Punch (H)";
//    controlTableRows[r]->description = "Highest byte of Punch";
//    controlTableRows[r]->writeAccess = true;
}
