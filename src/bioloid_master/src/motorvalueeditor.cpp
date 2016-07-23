#include "motorvalueeditor.h"
#include <iostream>
#include <sstream>
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QString>
#include <qt5/QtCore/QMapIterator>
#include <qt5/QtCore/QSignalMapper>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QFormLayout>
#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QAbstractSpinBox>
#include <qt5/QtWidgets/QFrame>
#include <qt5/QtWidgets/QMessageBox>
#include "commonvars.h"
#include "../../usb2ax_controller/src/ax12ControlTableMacros.h"


MotorValueEditor::MotorValueEditor(RosWorker* rosWorker, QWidget* parent) :
    mRosWorker(rosWorker), QWidget(parent), mSelectedControlTableAddress(1000)
{
    setWindowTitle("Motor Value Editor");

    int row = 0;
    int col = 0;

    populateMap(&optionsMap);
    optionsComboBox = new QComboBox;
    QMapIterator<QString, int> it(optionsMap);
    while (it.hasNext())
    {
        it.next();
        optionsComboBox->addItem(it.key());
    }
    optionsComboBox->setCurrentIndex(23);

    QFormLayout* formLayout = new QFormLayout;
    formLayout->addRow("Select\nwrite\noption:", optionsComboBox);

    QFrame* hlineFrame = new QFrame;
    hlineFrame->setFrameShape(QFrame::HLine);

    QGridLayout* layout = new QGridLayout;
    layout->addLayout(formLayout, row++, col, 1, -1);
    layout->addWidget(hlineFrame, row++, col, 1, -1);

    QLabel* motorLabel = new QLabel("Motor");
    QLabel* currentValueLabel = new QLabel("Current");
    QLabel* getValueLabel = new QLabel("Read");
    QLabel* goalValueLabel = new QLabel("Goal");
    QLabel* setValueLabel = new QLabel("Write");

    motorLabel->setAlignment(Qt::AlignCenter);
    currentValueLabel->setAlignment(Qt::AlignCenter);
    getValueLabel->setAlignment(Qt::AlignCenter);
    goalValueLabel->setAlignment(Qt::AlignCenter);
    setValueLabel->setAlignment(Qt::AlignCenter);

    col = 0;
    layout->addWidget(motorLabel, row, col++);
    layout->addWidget(currentValueLabel, row, col++);
    layout->addWidget(getValueLabel, row, col++);
    layout->addWidget(goalValueLabel, row, col++);
    layout->addWidget(setValueLabel, row++, col++);

    QSignalMapper* getSignalMapper = new QSignalMapper(this);
    QSignalMapper* setSignalMapper = new QSignalMapper(this);

    // + 1, for setting all values at once
    QVector<QLabel*> motorIdLabels(NUM_OF_MOTORS + 1);
    currentValueLineEdits.resize(NUM_OF_MOTORS + 1);
    goalValueSpinBoxes.resize(NUM_OF_MOTORS + 1);
    getValueButtons.resize(NUM_OF_MOTORS + 1);
    setValueButtons.resize(NUM_OF_MOTORS + 1);
    for (int i = 0; i <= NUM_OF_MOTORS; ++i)
    {
        std::ostringstream oss;
        oss.precision(3);
        oss.width(7);
        oss.fill(' ');
        oss.setf(std::ios::fixed, std::ios::floatfield);
        oss << 0.0;
        QString str = QString::fromStdString(oss.str());

        motorIdLabels[i] = new QLabel;
        currentValueLineEdits[i] = new QLineEdit;
        goalValueSpinBoxes[i] = new QDoubleSpinBox;
        goalValueSpinBoxes[i]->setRange(-2.56, 2.555);
        goalValueSpinBoxes[i]->setDecimals(3);
        goalValueSpinBoxes[i]->setSingleStep(0.01);
        getValueButtons[i] = new QPushButton("Get");
        setValueButtons[i] = new QPushButton("Set");

        motorIdLabels[i]->setAlignment(Qt::AlignCenter);
        currentValueLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        goalValueSpinBoxes[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        row = i + 3;
        col = 0;

        connect( getValueButtons[i], SIGNAL(clicked()),
                 getSignalMapper, SLOT(map()) );

        connect( setValueButtons[i], SIGNAL(clicked()),
                 setSignalMapper, SLOT(map()) );

        if (i != NUM_OF_MOTORS)
        {
            motorIdLabels[i]->setText(QString::number(i + 1));
            currentValueLineEdits[i]->setText(str);
            currentValueLineEdits[i]->setReadOnly(true);

            getSignalMapper->setMapping(getValueButtons[i], i + 1);
            setSignalMapper->setMapping(setValueButtons[i], i + 1);
        }
        else
        {
            motorIdLabels[i]->setText("All");
            currentValueLineEdits[i]->setReadOnly(true);

            QFrame* hlineFrame = new QFrame;
            hlineFrame->setFrameShape(QFrame::HLine);
            layout->addWidget(hlineFrame, row++, col, 1, -1);

            getSignalMapper->setMapping(getValueButtons[i], 254);
            setSignalMapper->setMapping(setValueButtons[i], 254);  // 254: Broadcast ID
        }

        layout->addWidget(motorIdLabels[i], row, col++);
        layout->addWidget(currentValueLineEdits[i], row, col++);
        layout->addWidget(getValueButtons[i], row, col++);
        layout->addWidget(goalValueSpinBoxes[i], row, col++);
        layout->addWidget(setValueButtons[i], row, col++);
    }

    setLayout(layout);

    connect( optionsComboBox, SIGNAL(currentIndexChanged(const QString &)),
             this, SLOT(updateOption(const QString &)) );
    connect( getSignalMapper, SIGNAL(mapped(int)), this, SLOT(getValue(int)) );
    connect( setSignalMapper, SIGNAL(mapped(int)), this, SLOT(setValue(int)) );
}


void MotorValueEditor::updateOption(const QString &text)
{
    if ( optionsMap.contains(text) )
        mSelectedControlTableAddress = optionsMap.value(text);

    // Update spin box input range
    switch (mSelectedControlTableAddress)
    {
    case 1000:  // Set goal position in rad
    {
        for (int i = 0; i < goalValueSpinBoxes.size(); ++i)
        {
            goalValueSpinBoxes[i]->setRange(-2.56, 2.555);
            goalValueSpinBoxes[i]->setDecimals(3);
            goalValueSpinBoxes[i]->setSingleStep(0.01);
        }
        break;
    }
    case 1001:  // Set goal speed in rad/sec
    {
        for (int i = 0; i < goalValueSpinBoxes.size(); ++i)
        {
            goalValueSpinBoxes[i]->setRange(-12.276, 12.276);
            goalValueSpinBoxes[i]->setDecimals(3);
            goalValueSpinBoxes[i]->setSingleStep(0.01);
        }
        break;
    }
    case 1002:  // Set max torque in decimal
    {
        for (int i = 0; i < goalValueSpinBoxes.size(); ++i)
        {
            goalValueSpinBoxes[i]->setRange(-1.023, 1.023);
            goalValueSpinBoxes[i]->setDecimals(3);
            goalValueSpinBoxes[i]->setSingleStep(0.01);
        }
        break;
    }
    default:
    {
        for (int i = 0; i < goalValueSpinBoxes.size(); ++i)
        {
            goalValueSpinBoxes[i]->setRange(0.0, 2047.0);
            goalValueSpinBoxes[i]->setDecimals(0);
            goalValueSpinBoxes[i]->setSingleStep(1.0);
        }
        break;
    }
    }
}


void MotorValueEditor::getValue(int dxlId)
{
    std::ostringstream oss;
    oss.precision(3);
    oss.width(7);
    oss.fill(' ');
    oss.setf(std::ios::fixed, std::ios::floatfield);
    QString str;

    switch (mSelectedControlTableAddress)
    {
    case 1000:  // Get goal position in rad
    {
        if ( (1 <= dxlId) && (dxlId <= 18) )
        {
            usb2ax_controller::GetMotorParam srv;
            srv.request.dxlID = dxlId;
            if ( mRosWorker->getMotorGoalPositionInRadClient.call(srv) )
            {
                oss << srv.response.value;
                str = QString::fromStdString(oss.str());
                currentValueLineEdits[dxlId - 1]->setText(str);
            }
        }
        else if (dxlId == 254)
        {
            usb2ax_controller::GetMotorParams srv;
            for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
                srv.request.dxlIDs.push_back(dxlId);
            if ( mRosWorker->getMotorGoalPositionsInRadClient.call(srv) )
            {
                if ( srv.response.values.size() >= NUM_OF_MOTORS )
                {
                    for (int i = 0; i < NUM_OF_MOTORS; ++i)
                    {
                        oss.str("");
                        oss.width(7);  // Not 'sticky'
                        oss << srv.response.values[i];
                        str = QString::fromStdString(oss.str());
                        currentValueLineEdits[i]->setText(str);
                    }
                }
            }
        }
        else
            return;

        break;
    }
    case 1001:  // Get goal speed in rad/sec
    {
        if ( (1 <= dxlId) && (dxlId <= 18) )
        {
            usb2ax_controller::GetMotorParam srv;
            srv.request.dxlID = dxlId;
            if ( mRosWorker->getMotorGoalSpeedInRadPerSecClient.call(srv) )
            {
                oss << srv.response.value;
                str = QString::fromStdString(oss.str());
                currentValueLineEdits[dxlId - 1]->setText(str);
            }
        }
        else if (dxlId == 254)
        {
            usb2ax_controller::GetMotorParams srv;
            for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
                srv.request.dxlIDs.push_back(dxlId);
            if ( mRosWorker->getMotorGoalSpeedsInRadPerSecClient.call(srv) )
            {
                if ( srv.response.values.size() >= NUM_OF_MOTORS )
                {
                    for (int i = 0; i < NUM_OF_MOTORS; ++i)
                    {
                        oss.str("");
                        oss.width(7);  // Not 'sticky'
                        oss << srv.response.values[i];
                        str = QString::fromStdString(oss.str());
                        currentValueLineEdits[i]->setText(str);
                    }
                }
            }
        }
        else
            return;

        break;
    }
    case 1002:  // Get max torque in decimal
    {
        if ( (1 <= dxlId) && (dxlId <= 18) )
        {
            usb2ax_controller::GetMotorParam srv;
            srv.request.dxlID = dxlId;
            if ( mRosWorker->getMotorTorqueLimitInDecimalClient.call(srv) )
            {
                oss << srv.response.value;
                str = QString::fromStdString(oss.str());
                currentValueLineEdits[dxlId - 1]->setText(str);
            }
        }
        else if (dxlId == 254)
        {
            usb2ax_controller::GetMotorParams srv;
            for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
                srv.request.dxlIDs.push_back(dxlId);
            if ( mRosWorker->getMotorTorqueLimitsInDecimalClient.call(srv) )
            {
                if ( srv.response.values.size() >= NUM_OF_MOTORS )
                {
                    for (int i = 0; i < NUM_OF_MOTORS; ++i)
                    {
                        oss.str("");
                        oss.width(7);  // Not 'sticky'
                        oss << srv.response.values[i];
                        str = QString::fromStdString(oss.str());
                        currentValueLineEdits[i]->setText(str);
                    }
                }
            }
        }
        else
            return;

        break;
    }
    case AX12_ID:
    case AX12_BAUD_RATE:
    case AX12_RETURN_DELAY_TIME:
    case AX12_CW_ANGLE_LIMIT_L:
    case AX12_CCW_ANGLE_LIMIT_L:
    case AX12_HIGH_LIMIT_TEMPERATURE:
    case AX12_LOW_LIMIT_VOLTAGE:
    case AX12_HIGH_LIMIT_VOLTAGE:
    case AX12_MAX_TORQUE_L:
    case AX12_STATUS_RETURN_LEVEL:
    case AX12_ALARM_LED:
    case AX12_ALARM_SHUTDOWN:
    case AX12_TORQUE_ENABLE:
    case AX12_LED:
    case AX12_CW_COMPLIANCE_MARGIN:
    case AX12_CCW_COMPLIANCE_MARGIN:
    case AX12_CW_COMPLIANCE_SLOPE:
    case AX12_CCW_COMPLIANCE_SLOPE:
    case AX12_GOAL_POSITION_L:
    case AX12_MOVING_SPEED_L:
    case AX12_TORQUE_LIMIT_L:
    case AX12_LOCK:
    case AX12_PUNCH_L:
    {
        if ( (1 <= dxlId) && (dxlId <= 18) )
        {
            usb2ax_controller::ReceiveFromAX srv;
            srv.request.dxlID = dxlId;
            srv.request.address = mSelectedControlTableAddress;
            if ( mRosWorker->receiveFromAXClient.call(srv) )
            {
                oss << srv.response.value;
                str = QString::fromStdString(oss.str());
                currentValueLineEdits[dxlId - 1]->setText(str);
            }
        }
        else if (dxlId == 254)
        {
            usb2ax_controller::ReceiveSyncFromAX srv;
            for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
                srv.request.dxlIDs.push_back(dxlId);
            srv.request.startAddress = mSelectedControlTableAddress;
            srv.request.numOfValuesPerMotor = 1;
            if ( mRosWorker->receiveSyncFromAXClient.call(srv) )
            {
                if ( srv.response.values.size() >= NUM_OF_MOTORS )
                {
                    for (int i = 0; i < NUM_OF_MOTORS; ++i)
                    {
                        oss.str("");
                        oss.width(7);  // Not 'sticky'
                        oss << srv.response.values[i];
                        str = QString::fromStdString(oss.str());
                        currentValueLineEdits[i]->setText(str);
                    }
                }
            }
        }
        else
            return;

        break;
    }
    default:
    {
        // Do nothing
        break;
    }
    }
}


void MotorValueEditor::setValue(int dxlId)
{
//    std::cout << "Dynamixel ID: " << dxlId << ", Current address: "
//              << mSelectedControlTableAddress << std::endl;

    switch (mSelectedControlTableAddress)
    {
    case 1000:  // Set goal position in rad
    {
        usb2ax_controller::SetMotorParam srv;
        srv.request.dxlID = dxlId;
        if ( (1 <= dxlId) && (dxlId <= 18) )
            srv.request.value = goalValueSpinBoxes[dxlId - 1]->value();
        else if (dxlId == 254)
            srv.request.value = goalValueSpinBoxes[18]->value();
        else
            return;
        mRosWorker->setMotorGoalPositionInRadClient.call(srv);

        break;
    }
    case 1001:  // Set goal speed in rad/sec
    {
        usb2ax_controller::SetMotorParam srv;
        srv.request.dxlID = dxlId;
        if ( (1 <= dxlId) && (dxlId <= 18) )
            srv.request.value = goalValueSpinBoxes[dxlId - 1]->value();
        else if (dxlId == 254)
            srv.request.value = goalValueSpinBoxes[18]->value();
        else
            return;
        mRosWorker->setMotorGoalSpeedInRadPerSecClient.call(srv);

        break;
    }
    case 1002:  // Set max torque in decimal
    {
        usb2ax_controller::SetMotorParam srv;
        srv.request.dxlID = dxlId;
        if ( (1 <= dxlId) && (dxlId <= 18) )
            srv.request.value = goalValueSpinBoxes[dxlId - 1]->value();
        else if (dxlId == 254)
            srv.request.value = goalValueSpinBoxes[18]->value();
        else
            return;
        mRosWorker->setMotorTorqueLimitInDecimalClient.call(srv);

        break;
    }
    case AX12_ID:
    {
        QMessageBox::StandardButton reply = QMessageBox::warning(
                    this, "Set value?",
                    "Are you sure you want to change the motor's ID?\nThis is usually a bad idea!",
                    QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (reply == QMessageBox::No)
            return;
        // Else, fall through to next case (no break)
    }
    case AX12_BAUD_RATE:
    {
        QMessageBox::StandardButton reply =
                QMessageBox::warning(
                    this, "Set value?",
                    "Are you sure you want to change the motor's baud rate?\nThis is usually a bad idea!",
                    QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (reply == QMessageBox::No)
            return;
        // Else, fall through to next case (no break)
    }
    case AX12_RETURN_DELAY_TIME:
    case AX12_CW_ANGLE_LIMIT_L:
    case AX12_CCW_ANGLE_LIMIT_L:
    case AX12_HIGH_LIMIT_TEMPERATURE:
    case AX12_LOW_LIMIT_VOLTAGE:
    case AX12_HIGH_LIMIT_VOLTAGE:
    case AX12_MAX_TORQUE_L:
    case AX12_STATUS_RETURN_LEVEL:
    case AX12_ALARM_LED:
    case AX12_ALARM_SHUTDOWN:
    case AX12_TORQUE_ENABLE:
    case AX12_LED:
    case AX12_CW_COMPLIANCE_MARGIN:
    case AX12_CCW_COMPLIANCE_MARGIN:
    case AX12_CW_COMPLIANCE_SLOPE:
    case AX12_CCW_COMPLIANCE_SLOPE:
    case AX12_GOAL_POSITION_L:
    case AX12_MOVING_SPEED_L:
    case AX12_TORQUE_LIMIT_L:
    case AX12_LOCK:
    case AX12_PUNCH_L:
    {
        usb2ax_controller::SendToAX srv;
        srv.request.dxlID = dxlId;
        srv.request.address = mSelectedControlTableAddress;
        if ( (1 <= dxlId) && (dxlId <= 18) )
            srv.request.value = goalValueSpinBoxes[dxlId - 1]->value();
        else if (dxlId == 254)
            srv.request.value = goalValueSpinBoxes[18]->value();
        else
            return;
        mRosWorker->sendtoAXClient.call(srv);

        break;
    }
    default:
    {
        // Do nothing
        break;
    }
    }
}


void MotorValueEditor::populateMap(QMap<QString, int>* inputMap)
{
    inputMap->insert("Goal position in rad", 1000);
    inputMap->insert("Goal speed in rad/sec", 1001);
    inputMap->insert("Max torque in decimal", 1002);
    inputMap->insert("AX12_ID", AX12_ID);
    inputMap->insert("AX12_BAUD_RATE", AX12_BAUD_RATE);
    inputMap->insert("AX12_RETURN_DELAY_TIME", AX12_RETURN_DELAY_TIME);
    inputMap->insert("AX12_CW_ANGLE_LIMIT_L", AX12_CW_ANGLE_LIMIT_L);
    inputMap->insert("AX12_CCW_ANGLE_LIMIT_L", AX12_CCW_ANGLE_LIMIT_L);
    inputMap->insert("AX12_HIGH_LIMIT_TEMPERATURE", AX12_HIGH_LIMIT_TEMPERATURE);
    inputMap->insert("AX12_LOW_LIMIT_VOLTAGE", AX12_LOW_LIMIT_VOLTAGE);
    inputMap->insert("AX12_HIGH_LIMIT_VOLTAGE", AX12_HIGH_LIMIT_VOLTAGE);
    inputMap->insert("AX12_MAX_TORQUE_L", AX12_MAX_TORQUE_L);
    inputMap->insert("AX12_STATUS_RETURN_LEVEL", AX12_STATUS_RETURN_LEVEL);
    inputMap->insert("AX12_ALARM_LED", AX12_ALARM_LED);
    inputMap->insert("AX12_ALARM_SHUTDOWN", AX12_ALARM_SHUTDOWN);
    inputMap->insert("AX12_TORQUE_ENABLE", AX12_TORQUE_ENABLE);
    inputMap->insert("AX12_LED", AX12_LED);
    inputMap->insert("AX12_CW_COMPLIANCE_MARGIN", AX12_CW_COMPLIANCE_MARGIN);
    inputMap->insert("AX12_CCW_COMPLIANCE_MARGIN", AX12_CCW_COMPLIANCE_MARGIN);
    inputMap->insert("AX12_CW_COMPLIANCE_SLOPE", AX12_CW_COMPLIANCE_SLOPE);
    inputMap->insert("AX12_CCW_COMPLIANCE_SLOPE", AX12_CCW_COMPLIANCE_SLOPE);
    inputMap->insert("AX12_GOAL_POSITION_L", AX12_GOAL_POSITION_L);
    inputMap->insert("AX12_MOVING_SPEED_L", AX12_MOVING_SPEED_L);
    inputMap->insert("AX12_TORQUE_LIMIT_L", AX12_TORQUE_LIMIT_L);
    inputMap->insert("AX12_LOCK", AX12_LOCK);
    inputMap->insert("AX12_PUNCH_L", AX12_PUNCH_L);
}
