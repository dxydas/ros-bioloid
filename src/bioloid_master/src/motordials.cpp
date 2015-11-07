#include "motordials.h"
#include <iostream>
#include <sstream>
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QString>
#include <qt5/QtCore/QSignalMapper>
#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QAbstractSlider>
#include "../usb2ax_controller/src/ax12ControlTableMacros.h"  // TODO: Fix this

#define NUM_OF_MOTORS 18


MotorDials::MotorDials(RosWorker* rosWorker, QWidget* parent) :
    mRosWorker(rosWorker), QWidget(parent), mSelectedMotor(-1)
{
    setWindowTitle("Motor Position Dials");

    QGridLayout* mainGridLayout = new QGridLayout;

    QSignalMapper* signalMapper = new QSignalMapper(this);

    groupBoxes.resize(NUM_OF_MOTORS);
    QVector<QGridLayout*> gridLayouts;
    gridLayouts.resize(NUM_OF_MOTORS);
    dials.resize(NUM_OF_MOTORS);
    presentPosLineEdits.resize(NUM_OF_MOTORS);
    presentSpeedLineEdits.resize(NUM_OF_MOTORS);

    int row;
    int col;
    for (int i = 0; i < NUM_OF_MOTORS; ++i)
    {
        groupBoxes[i] = new QGroupBox("Motor " + QString::number(i + 1));
        gridLayouts[i] = new QGridLayout;

        dials[i] = new QDial;
        dials[i]->setMinimum(-2606);
        dials[i]->setMaximum(2606);
        dials[i]->setWrapping(false);
        dials[i]->setNotchesVisible(true);
        //dials[i]->setNotchTarget(10);

        std::ostringstream oss;
        oss.precision(4);
        oss.width(8);
        oss.fill(' ');
        oss.setf(std::ios::fixed, std::ios::floatfield);
        oss << 0.0;
        QString str = QString::fromStdString(oss.str());
        presentPosLineEdits[i] = new QLineEdit(str);
        presentSpeedLineEdits[i] = new QLineEdit(str);

        connect( dials[i], SIGNAL(valueChanged(int)), signalMapper, SLOT(map()) );
        connect( dials[i], SIGNAL(valueChanged(int)), this, SLOT(setValue(int)) );

        signalMapper->setMapping(dials[i], i + 1);

        QLabel* presentPosLabel = new QLabel("Position:");
        QLabel* goalPosLabel = new QLabel("Speed:");

        row = 0;
        col = 0;
        gridLayouts[i]->addWidget(dials[i], row++, col, 1, -1, Qt::AlignCenter);
        gridLayouts[i]->addWidget(presentPosLabel, row, col++, 1, 1, Qt::AlignCenter);
        gridLayouts[i]->addWidget(goalPosLabel, row++, col--, 1, 1, Qt::AlignCenter);
        gridLayouts[i]->addWidget(presentPosLineEdits[i], row, col++, 1, 1, Qt::AlignCenter);
        gridLayouts[i]->addWidget(presentSpeedLineEdits[i], row, col--, 1, 1, Qt::AlignCenter);

        groupBoxes[i]->setLayout(gridLayouts[i]);

        int gridCols = 6;
        row = i / gridCols;
        col = i % gridCols;
        mainGridLayout->addWidget(groupBoxes[i], row, col);
    }

    setLayout(mainGridLayout);

    customiseLayout();

    callTime.start();

    connect( signalMapper, SIGNAL(mapped(int)), this, SLOT(setMotor(int)) );
    connect( mRosWorker, SIGNAL(jointStateUpdated(sensor_msgs::JointState)),
             this, SLOT(updateJointStateValues(sensor_msgs::JointState)) );
}


void MotorDials::setMotor(int dxlId)
{
//    std::cout << "Motor selected: " << dxlId << std::endl;

    mSelectedMotor = dxlId;
}


void MotorDials::setValue(int value)
{
    if ( callTime.elapsed() >= 100 )
    {
//        std::cout << "Motor selected: " << mSelectedMotor << std::endl;
//        std::cout << "Time: " << callTime.toString("hh:mm:ss ").toStdString() << std::endl;
//        std::cout << "Value: " << value / 1000.0 << std::endl;

    //    std::ostringstream oss;
    //    oss.precision(4);
    //    oss.width(8);
    //    oss.fill(' ');
    //    oss.setf(std::ios::fixed, std::ios::floatfield);
    //    oss << value / 1000.0;
    //    presentPosLineEdits[mSelectedMotor - 1]->setText(QString::fromStdString(oss.str()));

        usb2ax_controller::SetMotorParam srv;
        srv.request.dxlID = mSelectedMotor;
        if ( ((1 <= mSelectedMotor) && (mSelectedMotor <= 18)) )
            srv.request.value = value / 1000.0;
        else
            return;
        mRosWorker->setMotorGoalPositionInRadClient.call(srv);

        callTime.restart();
    }
}


void MotorDials::updateJointStateValues(sensor_msgs::JointState js)
{
    if ( (js.position.size() >= NUM_OF_MOTORS) )
    {
        for (int i = 0; i < NUM_OF_MOTORS; ++i)
        {
            std::ostringstream oss;
            oss.precision(4);
            oss.width(8);
            oss.fill(' ');
            oss.setf(std::ios::fixed, std::ios::floatfield);
            oss << js.position[i];
            presentPosLineEdits[i]->setText( QString::fromStdString(oss.str()) );

            oss.str("");
            oss.width(8);  // Not 'sticky'
            oss << js.velocity[i];
            presentSpeedLineEdits[i]->setText( QString::fromStdString(oss.str()) );
        }
    }
}


void MotorDials::initialiseDials(bool visible)
{
    if (visible)
    {
        sensor_msgs::JointState js = mRosWorker->getCurrentJointState();
        if ( (js.position.size() >= NUM_OF_MOTORS) )
        {
            for (int i = 0; i < NUM_OF_MOTORS; ++i)
            {
                dials[i]->setValue( static_cast<float>(js.position[i])*1000 );
            }
        }
    }
}


void MotorDials::customiseLayout()
{
    QString groupBoxStyleSheet =
            ( "QGroupBox {"
              //"background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 silver, stop: 1 white);"
              "border: 2px solid gray;"
              "border-radius: 5px;"
              "margin-top: 1.5ex; }"
              "QGroupBox::title {"
              "subcontrol-origin: margin;"
              "subcontrol-position: top center;"
              "padding: 0 3px;"
              "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 silver, stop: 1 white);"
              "border: 2px solid gray;"
              "border-radius: 5px; }" );

//    QString dialStyleSheet =
//            ( "QDial {"
//              "background-color: slategrey; }" );

    QString editBoxStyleSheet =
            ( "background-color: lightslategrey;" );

    for (int i = 0; i < NUM_OF_MOTORS; ++i)
    {
        groupBoxes[i]->setStyleSheet(groupBoxStyleSheet);
        //dials[i]->setStyleSheet(dialStyleSheet);
        presentPosLineEdits[i]->setStyleSheet(editBoxStyleSheet);
        presentSpeedLineEdits[i]->setStyleSheet(editBoxStyleSheet);
    }
}
