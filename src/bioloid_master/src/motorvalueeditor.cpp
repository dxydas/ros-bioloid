#include "motorvalueeditor.h"
#include <iostream>
#include <sstream>
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QMapIterator>
#include <qt5/QtCore/QSignalMapper>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QAbstractSpinBox>
#include <qt5/QtWidgets/QFrame>
#include "usb2ax_controller/SendToAX.h"
#include "usb2ax_controller/SendSyncToAX.h"
#include "usb2ax_controller/SetMotorParam.h"
#include "../usb2ax_controller/src/ax12ControlTableMacros.h"  // TODO: Fix this

#define NUM_OF_MOTORS 18


MotorValueEditor::MotorValueEditor(RosWorker* rosWorker, QWidget* parent) :
    mRosWorker(rosWorker), QWidget(parent), mSelectedControlTableAddress(1000)
{
    setWindowTitle("Motor Value Editor");

    int row = 0;
    int col = 0;

    QLabel* optionLabel = new QLabel("Select\nwrite\noption:");
    //
    populateMap(&optionsMap);
    optionsComboBox = new QComboBox;
    QMapIterator<QString, int> it(optionsMap);
    while (it.hasNext())
    {
        it.next();
        optionsComboBox->addItem(it.key());
    }
    optionsComboBox->setCurrentText("Position in rad");
    //
    QFrame* hlineFrame = new QFrame;
    hlineFrame->setFrameShape(QFrame::HLine);

    QGridLayout* layout = new QGridLayout;
    layout->addWidget(optionLabel, row, col++);
    layout->addWidget(optionsComboBox, row++, col--, 1, -1);
    layout->addWidget(hlineFrame, row++, col, 1, -1);

    QLabel* motorLabel = new QLabel("Motor");
    QLabel* currentValueLabel = new QLabel("Current");
    QLabel* goalValueLabel = new QLabel("Goal");
    QLabel* setValueLabel = new QLabel("Apply");

    motorLabel->setAlignment(Qt::AlignCenter);
    currentValueLabel->setAlignment(Qt::AlignCenter);
    goalValueLabel->setAlignment(Qt::AlignCenter);
    setValueLabel->setAlignment(Qt::AlignCenter);

    col = 0;
    layout->addWidget(motorLabel, row, col++);
    layout->addWidget(currentValueLabel, row, col++);
    layout->addWidget(goalValueLabel, row, col++);
    layout->addWidget(setValueLabel, row++, col++);

    QSignalMapper* signalMapper = new QSignalMapper(this);

    // + 1, for setting all values at once
    QVector<QLabel*> motorIdLabels(NUM_OF_MOTORS + 1);
    currentValueLineEdits.resize(NUM_OF_MOTORS + 1);
    goalValueSpinBoxes.resize(NUM_OF_MOTORS + 1);
    setValueButtons.resize(NUM_OF_MOTORS + 1);
    for (int i = 0; i <= NUM_OF_MOTORS; ++i)
    {
        std::ostringstream oss;
        oss.precision(4);
        oss.width(8);
        oss.fill(' ');
        oss.setf(std::ios::fixed, std::ios::floatfield);
        oss << 0.0;
        QString str = QString::fromStdString(oss.str());

        motorIdLabels[i] = new QLabel;//QString::number(i+1));
        currentValueLineEdits[i] = new QLineEdit;//(str);
        goalValueSpinBoxes[i] = new QDoubleSpinBox;
        goalValueSpinBoxes[i]->setRange(-11.9, 11.9);
        goalValueSpinBoxes[i]->setDecimals(4);
        goalValueSpinBoxes[i]->setSingleStep(0.001);
        setValueButtons[i] = new QPushButton("Set");

        motorIdLabels[i]->setAlignment(Qt::AlignCenter);
        currentValueLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        goalValueSpinBoxes[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        row = i + 3;
        col = 0;

        connect( setValueButtons[i], SIGNAL(clicked()),
                 signalMapper, SLOT(map()) );

        if (i != NUM_OF_MOTORS)
        {
            motorIdLabels[i]->setText(QString::number(i + 1));
            currentValueLineEdits[i]->setText(str);

            signalMapper->setMapping(setValueButtons[i], i + 1);
        }
        else
        {
            motorIdLabels[i]->setText("All");
            currentValueLineEdits[i]->setEnabled(false);

            QFrame* hlineFrame = new QFrame;
            hlineFrame->setFrameShape(QFrame::HLine);
            layout->addWidget(hlineFrame, row++, col, 1, -1);

            signalMapper->setMapping(setValueButtons[i], 254);  // 254: Broadcast ID
        }

        layout->addWidget(motorIdLabels[i], row, col++);
        layout->addWidget(currentValueLineEdits[i], row, col++);
        layout->addWidget(goalValueSpinBoxes[i], row, col++);
        layout->addWidget(setValueButtons[i], row, col++);
    }

    setLayout(layout);

    customiseLayout();

    connect( optionsComboBox, SIGNAL(currentIndexChanged(const QString &)),
             this, SLOT(updateOption(const QString &)) );
    connect( signalMapper, SIGNAL(mapped(int)), this, SLOT(setValue(int)) );
}


void MotorValueEditor::updateOption(const QString &text)
{
    if ( optionsMap.contains(text) )
        mSelectedControlTableAddress = optionsMap.value(text);

    // Update spin box input range
    switch (mSelectedControlTableAddress)
    {
    case 1000:  // Set Position in rad
    {
        for (int i = 0; i < goalValueSpinBoxes.size(); ++i)
        {
            goalValueSpinBoxes[i]->setRange(-2.618, 2.618);
            goalValueSpinBoxes[i]->setDecimals(4);
            goalValueSpinBoxes[i]->setSingleStep(0.01);
        }
        break;
    }
    case 1001:  // Set Speed in rad/sec
    {
        for (int i = 0; i < goalValueSpinBoxes.size(); ++i)
        {
            goalValueSpinBoxes[i]->setRange(-11.8668, 11.8668);
            goalValueSpinBoxes[i]->setDecimals(4);
            goalValueSpinBoxes[i]->setSingleStep(0.01);
        }
        break;
    }
    case 1002:  // Set Torque in %
    {
        for (int i = 0; i < goalValueSpinBoxes.size(); ++i)
        {
            goalValueSpinBoxes[i]->setRange(0.0, 100.0);
            goalValueSpinBoxes[i]->setDecimals(4);
            goalValueSpinBoxes[i]->setSingleStep(1.0);
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


void MotorValueEditor::populateMap(QMap<QString, int>* inputMap)
{
    inputMap->insert("Position in rad", 1000);
    inputMap->insert("Speed in rad/sec", 1001);
    inputMap->insert("Torque in %", 1002);
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


void MotorValueEditor::setValue(int dxlId)
{
//    std::cout << "Dynamixel ID: " << dxlId << ", Current address: "
//              << mSelectedControlTableAddress << std::endl;

    switch (mSelectedControlTableAddress)
    {
    case 1000:  // Set Position in rad
    {
        usb2ax_controller::SetMotorParam srv;
        srv.request.dxlID = dxlId;
        if ( ((1 <= dxlId) && (dxlId <= 18)) )
            srv.request.value = goalValueSpinBoxes[dxlId - 1]->value();
        else if (dxlId == 254)
            srv.request.value = goalValueSpinBoxes[18]->value();
        else
            return;
        mRosWorker->setMotorGoalPositionInRadClient.call(srv);

        break;
    }
    case 1001:  // Set Speed in rad/sec
    {
        usb2ax_controller::SetMotorParam srv;
        srv.request.dxlID = dxlId;
        if ( ((1 <= dxlId) && (dxlId <= 18)) )
            srv.request.value = goalValueSpinBoxes[dxlId - 1]->value();
        else if (dxlId == 254)
            srv.request.value = goalValueSpinBoxes[18]->value();
        else
            return;
        mRosWorker->setMotorGoalSpeedInRadPerSecClient.call(srv);

        break;
    }
    case 1002:  // Set Torque in %
    {
        usb2ax_controller::SetMotorParam srv;
        srv.request.dxlID = dxlId;
        if ( ((1 <= dxlId) && (dxlId <= 18)) )
            srv.request.value = goalValueSpinBoxes[dxlId - 1]->value() / 100.0;  // Convert % to decimal
        else if (dxlId == 254)
            srv.request.value = goalValueSpinBoxes[18]->value() / 100.0;  // Convert % to decimal
        else
            return;
        mRosWorker->setMotorMaxTorqueInDecimalClient.call(srv);

        break;
    }
    case AX12_ID:
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
        if ( ((1 <= dxlId) && (dxlId <= 18)) )
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


void MotorValueEditor::customiseLayout()
{
    QString buttonStyleSheet =
            ( "QPushButton {"
              "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
              "stop: 0 lightsteelblue, stop: 1 steelblue);"
              "border: solid #8F8F91;"
              "border-style: outset;"
              "border-width: 4px;"
              "border-radius: 10px; }"
              //"border-color: beige; }"
              //"font: bold 14px; }"
              //"min-width: 10em;"
              //"padding: 6px; }"
              "QPushButton:flat {"
              "border: none;"  /* no border for a flat push button */
              "}"
              "QPushButton:default {"
              "border-color: navy;"  /* make the default button prominent */
              "}"
              "QPushButton:pressed {"
              "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
              "stop: 0 royalblue, stop: 1 dodgerblue);"
              "border-style: inset; }" );

    QString editBoxStyleSheet =
            ( "background-color: lightslategrey;" );

    optionsComboBox->setStyleSheet(editBoxStyleSheet);
    for (int i = 0; i <= NUM_OF_MOTORS; ++i)
    {
        currentValueLineEdits[i]->setStyleSheet(editBoxStyleSheet);
        goalValueSpinBoxes[i]->setStyleSheet(editBoxStyleSheet);
        setValueButtons[i]->setStyleSheet(buttonStyleSheet);
    }
}
