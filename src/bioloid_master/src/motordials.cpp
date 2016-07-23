#include "motordials.h"
#include <iostream>
#include <sstream>
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QObject>
#include <qt5/QtCore/QString>
#include <qt5/QtCore/QSignalMapper>
#include <qt5/QtWidgets/QHBoxLayout>
#include <qt5/QtWidgets/QVBoxLayout>
#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QAbstractButton>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QCheckBox>
#include "commonvars.h"
#include "../../usb2ax_controller/src/ax12ControlTableMacros.h"


MotorDials::MotorDials(RosWorker* rosWorker, QWidget* parent) :
    mRosWorker(rosWorker), QWidget(parent), mSelectedMotor(-1), dialsInitialised(false)
{
    setWindowTitle("Motor Position Dials");

    QSignalMapper* comboBoxSignalMapper = new QSignalMapper(this);
    QSignalMapper* dialSignalMapper = new QSignalMapper(this);

    QVector<QGridLayout*> gridLayouts;
    QVector<QGridLayout*> subGridLayouts1;
    QVector<QGridLayout*> subGridLayouts2;
    QVector<QGridLayout*> subGridLayouts3;
    QVector<QGridLayout*> subGridLayouts4;
    QVector<QGridLayout*> subGridLayouts5;

    QGridLayout* mainGridLayout = new QGridLayout;

    groupBoxes.resize(NUM_OF_MOTORS);
    gridLayouts.resize(NUM_OF_MOTORS);
    subGridLayouts1.resize(NUM_OF_MOTORS);
    subGridLayouts2.resize(NUM_OF_MOTORS);
    subGridLayouts3.resize(NUM_OF_MOTORS);
    subGridLayouts4.resize(NUM_OF_MOTORS);
    subGridLayouts5.resize(NUM_OF_MOTORS);
    dials.resize(NUM_OF_MOTORS);
    dialOptionComboBoxes.resize(NUM_OF_MOTORS);
    mSelectedDialOption.resize(NUM_OF_MOTORS);
    presentPosLineEdits.resize(NUM_OF_MOTORS);
    presentSpeedLineEdits.resize(NUM_OF_MOTORS);
    presentLoadLineEdits.resize(NUM_OF_MOTORS);
    goalValueLineEdits.resize(NUM_OF_MOTORS);
    torqueEnableLineEdits.resize(NUM_OF_MOTORS);
    ledLineEdits.resize(NUM_OF_MOTORS);
    presentVoltageLineEdits.resize(NUM_OF_MOTORS);
    presentTempLineEdits.resize(NUM_OF_MOTORS);
    alarmLedVectors.resize(NUM_OF_MOTORS);

    int row;
    int col;
    for (int i = 0; i < NUM_OF_MOTORS; ++i)
    {
        groupBoxes[i] = new QGroupBox("Motor " + QString::number(i + 1));
        gridLayouts[i] = new QGridLayout;
        subGridLayouts1[i] = new QGridLayout;
        subGridLayouts2[i] = new QGridLayout;
        subGridLayouts3[i] = new QGridLayout;
        subGridLayouts4[i] = new QGridLayout;
        subGridLayouts5[i] = new QGridLayout;

        std::ostringstream oss;
        oss.precision(3);
        oss.width(10);
        oss.fill(' ');
        oss.setf(std::ios::fixed, std::ios::floatfield);
        oss << 0.0;
        QString str = QString::fromStdString(oss.str());
        presentPosLineEdits[i] = new QLineEdit(str);
        presentSpeedLineEdits[i] = new QLineEdit(str);
        presentLoadLineEdits[i] = new QLineEdit(str);
        goalValueLineEdits[i] = new QLineEdit(str);
        torqueEnableLineEdits[i] = new QLineEdit("TORQUE");
        ledLineEdits[i] = new QLineEdit("LED");
        oss.str("");
        oss.precision(1);
        oss.width(3);
        oss << 0.0;
        str = QString::fromStdString(oss.str());
        presentVoltageLineEdits[i] = new QLineEdit(str + " V");
        oss.str("");
        oss.precision(0);
        oss.width(2);
        oss << 0.0;
        str = QString::fromStdString(oss.str());
        presentTempLineEdits[i] = new QLineEdit(str + " °C");

        presentPosLineEdits[i]->setReadOnly(true);
        presentSpeedLineEdits[i]->setReadOnly(true);
        presentLoadLineEdits[i]->setReadOnly(true);
        goalValueLineEdits[i]->setReadOnly(true);
        presentVoltageLineEdits[i]->setReadOnly(true);
        presentTempLineEdits[i]->setReadOnly(true);
        torqueEnableLineEdits[i]->setReadOnly(true);
        ledLineEdits[i]->setReadOnly(true);

        presentPosLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        presentSpeedLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        presentLoadLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        goalValueLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        presentVoltageLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        presentTempLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        torqueEnableLineEdits[i]->setAlignment(Qt::AlignCenter);
        ledLineEdits[i]->setAlignment(Qt::AlignCenter);

        presentPosLineEdits[i]->setToolTip("Position in rad");
        presentSpeedLineEdits[i]->setToolTip("Speed in rad/s");
        presentLoadLineEdits[i]->setToolTip("Load in decimal, as fraction of full value 1.0");
        presentVoltageLineEdits[i]->setToolTip("Present Voltage");
        presentTempLineEdits[i]->setToolTip("Present Temperature");
        torqueEnableLineEdits[i]->setToolTip("On/Off");
        ledLineEdits[i]->setToolTip("On/Off");

        alarmLedLineEdits.resize(7);
        alarmLedLineEdits[0] = new QLineEdit("IV");
        alarmLedLineEdits[1] = new QLineEdit("AL");
        alarmLedLineEdits[2] = new QLineEdit("OH");
        alarmLedLineEdits[3] = new QLineEdit("RG");
        alarmLedLineEdits[4] = new QLineEdit("CS");
        alarmLedLineEdits[5] = new QLineEdit("OL");
        alarmLedLineEdits[6] = new QLineEdit("IE");
        alarmLedLineEdits[0]->setToolTip("Input Voltage Error ");
        alarmLedLineEdits[1]->setToolTip("Angle Limit Error");
        alarmLedLineEdits[2]->setToolTip("OverHeating Error");
        alarmLedLineEdits[3]->setToolTip("Range Error");
        alarmLedLineEdits[4]->setToolTip("CheckSum Error");
        alarmLedLineEdits[5]->setToolTip("Overload Error");
        alarmLedLineEdits[6]->setToolTip("Instruction Error");
        for (int j = 0; j < alarmLedLineEdits.size(); ++j)
        {
            alarmLedLineEdits[j]->setReadOnly(true);
            alarmLedLineEdits[j]->setAlignment(Qt::AlignCenter);
            alarmLedLineEdits[j]->setMaximumSize(26, 30);
        }
        alarmLedVectors[i] = alarmLedLineEdits;

        dialOptionComboBoxes[i] = new QComboBox;
        dialOptionComboBoxes[i]->addItem("Dial: Position");
        dialOptionComboBoxes[i]->addItem("Dial: Speed");
        dialOptionComboBoxes[i]->addItem("Dial: Load");
        dialOptionComboBoxes[i]->setCurrentIndex(0);

        connect( dialOptionComboBoxes[i], SIGNAL(currentIndexChanged(int)), comboBoxSignalMapper, SLOT(map()) );
        connect( dialOptionComboBoxes[i], SIGNAL(currentIndexChanged(int)), this, SLOT(setDialOption(int)) );

        comboBoxSignalMapper->setMapping(dialOptionComboBoxes[i], i + 1);

        dials[i] = new QDial;

        connect( dials[i], SIGNAL(valueChanged(int)), dialSignalMapper, SLOT(map()) );
        connect( dials[i], SIGNAL(valueChanged(int)), this, SLOT(setValue(int)) );

        dialSignalMapper->setMapping(dials[i], i + 1);

        QLabel* presentPosLabel = new QLabel("Pos.:");
        QLabel* presentSpeedLabel = new QLabel("Spd:");
        QLabel* presentLoadLabel = new QLabel("Ld:");
        QLabel* goalValueLabel = new QLabel("Goal:");
        //QLabel* presentVoltageLabel = new QLabel("Volt.:");
        //QLabel* presentTempLabel = new QLabel("Temp.:");

        dials[i]->setMinimumSize(120, 100);
        dialOptionComboBoxes[i]->setMinimumWidth(120);
        presentPosLineEdits[i]->setMinimumWidth(55);
        goalValueLineEdits[i]->setMaximumWidth(70);
        torqueEnableLineEdits[i]->setMaximumWidth(65);
        ledLineEdits[i]->setMaximumWidth(65);
        presentVoltageLineEdits[i]->setMinimumWidth(55);
        presentTempLineEdits[i]->setMinimumWidth(55);

        row = 0;
        col = 0;
        subGridLayouts1[i]->addWidget(presentPosLabel, row, col++, 1, 1, Qt::AlignRight | Qt::AlignVCenter);
        subGridLayouts1[i]->addWidget(presentPosLineEdits[i], row++, col--, 1, 1, Qt::AlignCenter);
        subGridLayouts1[i]->addWidget(presentSpeedLabel, row, col++, 1, 1, Qt::AlignRight | Qt::AlignVCenter);
        subGridLayouts1[i]->addWidget(presentSpeedLineEdits[i], row++, col--, 1, 1, Qt::AlignCenter);
        subGridLayouts1[i]->addWidget(presentLoadLabel, row, col++, 1, 1, Qt::AlignRight | Qt::AlignVCenter);
        subGridLayouts1[i]->addWidget(presentLoadLineEdits[i], row++, col--, 1, 1, Qt::AlignCenter);

        row = 0;
        col = 0;
        subGridLayouts2[i]->addWidget(dialOptionComboBoxes[i], row++, col, 1, -1, Qt::AlignCenter);
        subGridLayouts2[i]->addWidget(dials[i], row++, col, 1, -1, Qt::AlignCenter);
        subGridLayouts2[i]->addWidget(goalValueLabel, row, col++, 1, 1, Qt::AlignRight | Qt::AlignVCenter);
        subGridLayouts2[i]->addWidget(goalValueLineEdits[i], row++, col, 1, 1, Qt::AlignCenter);

        row = 0;
        col = 0;
        //subGridLayouts3[i]->addWidget(presentVoltageLabel, row, col++, 1, 1, Qt::AlignRight | Qt::AlignVCenter);
        subGridLayouts3[i]->addWidget(presentVoltageLineEdits[i], row++, col, 1, 1, Qt::AlignCenter);
        //subGridLayouts3[i]->addWidget(presentTempLabel, row, col++, 1, 1, Qt::AlignRight | Qt::AlignVCenter);
        subGridLayouts3[i]->addWidget(presentTempLineEdits[i], row++, col, 1, 1, Qt::AlignCenter);

        row = 0;
        col = 0;
        subGridLayouts4[i]->addWidget(torqueEnableLineEdits[i], row++, col, 1, 1, Qt::AlignCenter);
        subGridLayouts4[i]->addWidget(ledLineEdits[i], row++, col, 1, 1, Qt::AlignCenter);
        subGridLayouts4[i]->setSpacing(1);

        row = 0;
        col = 0;
        for (int j = alarmLedLineEdits.size() - 1; j >= 0; --j)
        {
            subGridLayouts5[i]->addWidget(alarmLedVectors[i][j], row, col++, -1, 1, Qt::AlignCenter);
            subGridLayouts5[i]->setSpacing(0);
        }

        row = 0;
        col = 0;
        gridLayouts[i]->addLayout(subGridLayouts1[i], row, col++, 1, 1, Qt::AlignCenter);
        gridLayouts[i]->addLayout(subGridLayouts2[i], row, col++, 1, 1, Qt::AlignCenter);
        gridLayouts[i]->addLayout(subGridLayouts3[i], row++, col++, 1, 1, Qt::AlignCenter);
        col = 0;
        gridLayouts[i]->addLayout(subGridLayouts4[i], row, col++, 1, 1, Qt::AlignCenter);
        gridLayouts[i]->addLayout(subGridLayouts5[i], row, col++, 1, -1, Qt::AlignRight | Qt::AlignVCenter);

        groupBoxes[i]->setLayout(gridLayouts[i]);

        int gridCols = 6;
        row = i / gridCols;
        col = i % gridCols;
        mainGridLayout->addWidget(groupBoxes[i], row, col);
    }

    QPushButton* refreshButton = new QPushButton("Refresh");

    QCheckBox* autoRefreshCheckBox = new QCheckBox("Auto-Refresh (1 Hz)");

    QHBoxLayout* buttonsLayout = new QHBoxLayout;
    buttonsLayout->addWidget(refreshButton);
    buttonsLayout->addWidget(autoRefreshCheckBox);
    buttonsLayout->addStretch();  // Prevent widgets from filling horizontal space

    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(mainGridLayout);
    mainLayout->addLayout(buttonsLayout);
    setLayout(mainLayout);

    customiseLayout();

    callTime.start();

    feedbackTimer = new QTimer(this);

    connect( comboBoxSignalMapper, SIGNAL(mapped(int)), this, SLOT(setMotor(int)) );
    connect( dialSignalMapper, SIGNAL(mapped(int)), this, SLOT(setMotor(int)) );
    connect( mRosWorker, SIGNAL(jointStateUpdated(sensor_msgs::JointState)),
             this, SLOT(updateJointStateValues(sensor_msgs::JointState)) );
    connect( refreshButton, SIGNAL(clicked()), this, SLOT(updateLineEdits()) );
    connect( autoRefreshCheckBox, SIGNAL(stateChanged(int)), this, SLOT(toggleAutoRefresh(int)) );
    connect( feedbackTimer, SIGNAL(timeout()), this, SLOT(updateLineEdits()) );
}


void MotorDials::initialiseDials(bool visible)
{
    if (visible && !dialsInitialised)
    {
        // Updating the current index of each combo box triggers setDialOption()
        for (int i = 0; i < NUM_OF_MOTORS; ++i)
        {
            dialOptionComboBoxes[i]->setCurrentIndex(-1);
            dialOptionComboBoxes[i]->setCurrentIndex(0);
        }

//        sensor_msgs::JointState js = mRosWorker->getCurrentJointState();
//        if ( (js.position.size() >= NUM_OF_MOTORS) )
//        {
//            for (int i = 0; i < NUM_OF_MOTORS; ++i)
//            {
//                dials[i]->setValue( static_cast<float>(js.position[i])*1000 );
//            }
//        }

        dialsInitialised = true;
    }
}


void MotorDials::setMotor(int dxlId)
{
//    std::cout << "Motor selected: " << dxlId << std::endl;

    mSelectedMotor = dxlId;
}


void MotorDials::setDialOption(int index)
{
    if ( (0 <= index) && (index < dialOptionComboBoxes[0]->count()) )
    {
        if ( ((1 <= mSelectedMotor) && (mSelectedMotor <= 18)) )
        {
            int i = mSelectedMotor - 1;

            // Temporarily disconnect signal-slot to avoid calling setValue() while the dial option is changing
            disconnect( dials[i], SIGNAL(valueChanged(int)), this, SLOT(setValue(int)) );

            mSelectedDialOption[i] = index;

            sensor_msgs::JointState js = mRosWorker->getGoalJointState();

            dials[i]->setWrapping(false);
            dials[i]->setNotchesVisible(true);

            std::ostringstream oss;
            oss.precision(3);
            oss.width(7);
            oss.fill(' ');
            oss.setf(std::ios::fixed, std::ios::floatfield);

            switch (index)
            {
            case 0:
            {
                // Position
                // -2.56..2.555 rad, converted to -2560..2555 int range
                dials[i]->setRange(-2560, 2555);
                if ( (js.position.size() >= NUM_OF_MOTORS) )
                {
                    dials[i]->setValue( static_cast<float>(js.position[i])*1000 );
                    oss << js.position[i];
                    goalValueLineEdits[i]->setText( QString::fromStdString(oss.str()) );
                }
                break;
            }
            case 1:
            {
                // Speed
                // -12.276..12.276 rad/s, converted to -1227..1227 int range
                dials[i]->setRange(-1227, 1227);
                if ( (js.position.size() >= NUM_OF_MOTORS) )
                {
                    dials[i]->setValue( static_cast<float>(js.velocity[i])*1000 );
                    oss << js.velocity[i];
                    goalValueLineEdits[i]->setText( QString::fromStdString(oss.str()) );
                }
                break;
            }
            case 2:
            {
                // Load
                // -1.023..1.023 torque, converted to -1023..1023 int range
                dials[i]->setRange(-1023, 1023);
                if ( (js.position.size() >= NUM_OF_MOTORS) )
                {
                    dials[i]->setValue( static_cast<float>(js.effort[i])*1000 );
                    oss << js.effort[i];
                    goalValueLineEdits[i]->setText( QString::fromStdString(oss.str()) );
                }
                break;
            }
            default:
            {
                break;
            }
            }

            // Re-connect signal-slot
            connect( dials[i], SIGNAL(valueChanged(int)), this, SLOT(setValue(int)) );
        }
    }
}


void MotorDials::setValue(int value)
{
    if ( callTime.elapsed() >= 100 )
    {
//        std::cout << "Motor selected: " << mSelectedMotor << std::endl;
//        std::cout << "Time: " << callTime.toString("hh:mm:ss ").toStdString() << std::endl;
//        std::cout << "Value: " << value / 1000.0 << std::endl;

    //    std::ostringstream oss;
    //    oss.precision(3);
    //    oss.width(7);
    //    oss.fill(' ');
    //    oss.setf(std::ios::fixed, std::ios::floatfield);
    //    oss << value / 1000.0;
    //    presentPosLineEdits[mSelectedMotor - 1]->setText(QString::fromStdString(oss.str()));

        if ( ((1 <= mSelectedMotor) && (mSelectedMotor <= 18)) )
        {
            int i = mSelectedMotor - 1;

            usb2ax_controller::SetMotorParam srv;
            srv.request.dxlID = mSelectedMotor;
            srv.request.value = value / 1000.0;

            usb2ax_controller::GetMotorParam srv2;
            srv2.request.dxlID = mSelectedMotor;

            std::ostringstream oss;
            oss.precision(3);
            oss.width(7);
            oss.fill(' ');
            oss.setf(std::ios::fixed, std::ios::floatfield);

            switch (mSelectedDialOption[i])
            {
            case 0:
            {
                // Position
                if (mRosWorker->setMotorGoalPositionInRadClient.call(srv))
                {
                    if (mRosWorker->getMotorGoalPositionInRadClient.call(srv2))
                    {
                        oss << srv2.response.value;
                        goalValueLineEdits[i]->setText( QString::fromStdString(oss.str()) );
                    }
                }
                break;
            }
            case 1:
            {
                // Speed
                if (mRosWorker->setMotorGoalSpeedInRadPerSecClient.call(srv))
                {
                    if (mRosWorker->getMotorGoalSpeedInRadPerSecClient.call(srv2))
                    {
                        oss << srv2.response.value;
                        goalValueLineEdits[i]->setText( QString::fromStdString(oss.str()) );
                    }
                }
                break;
            }
            case 2:
            {
                // Load
                if (mRosWorker->setMotorTorqueLimitInDecimalClient.call(srv))
                {
                    if (mRosWorker->getMotorTorqueLimitInDecimalClient.call(srv2))
                    {
                        oss << srv2.response.value;
                        goalValueLineEdits[i]->setText( QString::fromStdString(oss.str()) );
                    }
                }
                break;
            }
            default:
            {
                return;
            }
            }

            callTime.restart();
        }
    }
}


void MotorDials::updateJointStateValues(sensor_msgs::JointState js)
{
    if ( (js.position.size() >= NUM_OF_MOTORS) )
    {
        for (int i = 0; i < NUM_OF_MOTORS; ++i)
        {
            std::ostringstream oss;
            oss.precision(3);
            oss.width(7);
            oss.fill(' ');
            oss.setf(std::ios::fixed, std::ios::floatfield);
            oss << js.position[i];
            presentPosLineEdits[i]->setText( QString::fromStdString(oss.str()) );

            oss.str("");
            oss.width(7);  // Not 'sticky'
            oss << js.velocity[i];
            presentSpeedLineEdits[i]->setText( QString::fromStdString(oss.str()) );

            oss.str("");
            oss.width(7);  // Not 'sticky'
            oss << js.effort[i];
            presentLoadLineEdits[i]->setText( QString::fromStdString(oss.str()) );
        }
    }
}


void MotorDials::updateLineEdits()
{
    usb2ax_controller::ReceiveSyncFromAX srv;
    for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
        srv.request.dxlIDs.push_back(dxlId);

    std::ostringstream oss;
    oss.fill(' ');
    oss.setf(std::ios::fixed, std::ios::floatfield);

    // Voltage
    srv.request.startAddress = AX12_PRESENT_VOLTAGE;
    srv.request.numOfValuesPerMotor = 1;
    if (mRosWorker->receiveSyncFromAXClient.call(srv))
    {
        if (srv.response.values.size() >= presentVoltageLineEdits.size())
        {
            for (int i = 0; i < presentVoltageLineEdits.size(); ++i)
            {
                oss.str("");
                oss.precision(1);
                oss.width(3);
                oss << srv.response.values[i] / 10.0;
                presentVoltageLineEdits[i]->setText( QString::fromStdString(oss.str()) + " V" );
            }
        }
    }

    // Temperature
    srv.request.startAddress = AX12_PRESENT_TEMPERATURE;
    srv.request.numOfValuesPerMotor = 1;
    if (mRosWorker->receiveSyncFromAXClient.call(srv))
    {
        if (srv.response.values.size() >= presentTempLineEdits.size())
        {
            for (int i = 0; i < presentTempLineEdits.size(); ++i)
            {
                oss.str("");
                oss.precision(0);
                oss.width(2);
                oss << srv.response.values[i];
                presentTempLineEdits[i]->setText( QString::fromStdString(oss.str()) + " °C" );
            }
        }
    }

    // Torque state
    srv.request.startAddress = AX12_TORQUE_ENABLE;
    srv.request.numOfValuesPerMotor = 1;
    if (mRosWorker->receiveSyncFromAXClient.call(srv))
    {
        if (srv.response.values.size() >= torqueEnableLineEdits.size())
        {
            for (int i = 0; i < torqueEnableLineEdits.size(); ++i)
            {
                if (srv.response.values[i])
                    torqueEnableLineEdits[i]->setStyleSheet(ledOnLineEdit);
                else
                    torqueEnableLineEdits[i]->setStyleSheet(ledOffLineEdit);
            }
        }
    }

    // LED state
    srv.request.startAddress = AX12_LED;
    srv.request.numOfValuesPerMotor = 1;
    if (mRosWorker->receiveSyncFromAXClient.call(srv))
    {
        if (srv.response.values.size() >= ledLineEdits.size())
        {
            for (int i = 0; i < ledLineEdits.size(); ++i)
            {
                if (srv.response.values[i])
                    ledLineEdits[i]->setStyleSheet(ledOnLineEdit);
                else
                    ledLineEdits[i]->setStyleSheet(ledOffLineEdit);
            }
        }
    }

    // Alarm LEDs states
    srv.request.startAddress = AX12_ALARM_LED;
    srv.request.numOfValuesPerMotor = 1;
    if (mRosWorker->receiveSyncFromAXClient.call(srv))
    {
        if (srv.response.values.size() >= alarmLedVectors.size())
        {
            for (int i = 0; i < alarmLedVectors.size(); ++i)
            {
                for (int j = 0; j < alarmLedVectors[i].size(); ++j)
                {
                    if ( srv.response.values[i] & (int)pow(2, j) )  // Extract bit value
                        alarmLedVectors[i][j]->setStyleSheet(ledArrayOnLineEdit);
                    else
                        alarmLedVectors[i][j]->setStyleSheet(ledArrayOffLineEdit);
                }
            }
        }
    }

}


void MotorDials::toggleAutoRefresh(int toggle)
{
    if (toggle)
        feedbackTimer->start(1000);
    else
        feedbackTimer->stop();
}


void MotorDials::customiseLayout()
{
    ledOffLineEdit =
            " QLineEdit { "
            "     border: 2px solid goldenrod; "
            "     background-color: darkred; "
            " } "
            " QLineEdit:disabled { "
            "     color: black; "
            " } ";

    ledOnLineEdit =
            " QLineEdit { "
            "     border: 2px solid goldenrod; "
            "     background-color: red; "
            " } "
            " QLineEdit:disabled { "
            "     color: white; "
            " } ";

    ledArrayOffLineEdit =
            " QLineEdit { "
            "     font: 10px; "
            "     margin: 0px; "
            "     border: 2px solid black; "
            "     background-color: darkred; "
            " } "
            " QLineEdit:disabled { "
            "     color: black; "
            " } ";

    ledArrayOnLineEdit =
            " QLineEdit { "
            "     font: 10px; "
            "     margin: 0px; "
            "     border: 2px solid black; "
            "     background-color: red; "
            " } "
            " QLineEdit:disabled { "
            "     color: white; "
            " } ";

    for (int i = 0; i < NUM_OF_MOTORS; ++i)
    {
        goalValueLineEdits[i]->setObjectName("silverLineEdit");
        presentVoltageLineEdits[i]->setObjectName("greenLineEdit");
        presentTempLineEdits[i]->setObjectName("greenLineEdit");
        torqueEnableLineEdits[i]->setStyleSheet(ledOffLineEdit);
        ledLineEdits[i]->setStyleSheet(ledOffLineEdit);
        for (int j = 0; j < alarmLedVectors[i].size(); ++j)
            alarmLedVectors[i][j]->setStyleSheet(ledArrayOffLineEdit);
    }
}
