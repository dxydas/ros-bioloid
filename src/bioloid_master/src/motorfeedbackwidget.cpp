#include "motorfeedbackwidget.h"
#include <iostream>
#include <sstream>
#include <qt5/QtCore/QString>
#include <qt5/QtCore/QVector>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QFrame>
#include "commonvars.h"


MotorFeedbackWidget::MotorFeedbackWidget(QWidget* parent) :
    QWidget(parent)
{
    QLabel* motorLabel = new QLabel("Motor");
    QLabel* presentPositionAndSelectedPoseLabel = new QLabel("Present position\nand selected pose");
    QLabel* presentPositionLabel = new QLabel("Present\nposition");
    QLabel* goalPositionLabel = new QLabel("Goal\nposition");
    QLabel* presentSpeedLabel = new QLabel("Present\nspeed");
    QLabel* movingSpeedLabel = new QLabel("Moving\nspeed");

    motorLabel->setAlignment(Qt::AlignCenter);
    presentPositionAndSelectedPoseLabel->setAlignment(Qt::AlignCenter);
    presentPositionLabel->setAlignment(Qt::AlignCenter);
    goalPositionLabel->setAlignment(Qt::AlignCenter);
    presentSpeedLabel->setAlignment(Qt::AlignCenter);
    movingSpeedLabel->setAlignment(Qt::AlignCenter);

    presentPositionAndSelectedPoseLabel->setObjectName("royalBlueLabel");
    presentPositionLabel->setObjectName("royalBlueLabel");
    goalPositionLabel->setObjectName("midnightBlueLabel");
    presentSpeedLabel->setObjectName("royalBlueLabel");
    movingSpeedLabel->setObjectName("midnightBlueLabel");

    QGridLayout* motorFeedbackLayout = new QGridLayout;
    int row = 0;
    int col = 0;
    motorFeedbackLayout->addWidget(motorLabel, row, col++);
    ++col;  // Leave a blank column in order to add vline spacer later
    motorFeedbackLayout->addWidget(presentPositionAndSelectedPoseLabel, row, col++);
    motorFeedbackLayout->addWidget(presentPositionLabel, row, col++);
    ++col;  // For vline
    motorFeedbackLayout->addWidget(goalPositionLabel, row, col++);
    ++col;  // For vline
    motorFeedbackLayout->addWidget(presentSpeedLabel, row, col++);
    ++col;  // For vline
    motorFeedbackLayout->addWidget(movingSpeedLabel, row, col++);

    QVector<QLabel*> motorIdLabels(NUM_OF_MOTORS);
    presentPosSliders.resize(NUM_OF_MOTORS);
    presentPosLineEdits.resize(NUM_OF_MOTORS);
    goalPosLineEdits.resize(NUM_OF_MOTORS);
    presentSpeedLineEdits.resize(NUM_OF_MOTORS);
    movingSpeedLineEdits.resize(NUM_OF_MOTORS);
    for (int i = 0; i < NUM_OF_MOTORS; ++i)
    {
        motorIdLabels[i] = new QLabel(QString::number(i + 1));
        presentPosSliders[i] = new DoubleSlider(Qt::Horizontal, this);
        presentPosSliders[i]->setRange(-2560, 2555);
        presentPosSliders[i]->setValue(0);
        presentPosSliders[i]->setSecondValue(0);

        std::ostringstream oss;
        oss.precision(3);
        oss.width(7);
        oss.fill(' ');
        oss.setf(std::ios::fixed, std::ios::floatfield);
        oss << 0.0;
        QString str = QString::fromStdString(oss.str());

        presentPosLineEdits[i] = new QLineEdit(str);
        goalPosLineEdits[i] = new QLineEdit(str);
        presentSpeedLineEdits[i] = new QLineEdit(str);
        movingSpeedLineEdits[i] = new QLineEdit(str);
        int maxLineEditWidth = 60;
        int maxLineEditHeight = 16;
        presentPosLineEdits[i]->setMaximumSize(maxLineEditWidth, maxLineEditHeight);
        goalPosLineEdits[i]->setMaximumSize(maxLineEditWidth, maxLineEditHeight);
        presentSpeedLineEdits[i]->setMaximumSize(maxLineEditWidth, maxLineEditHeight);
        movingSpeedLineEdits[i]->setMaximumSize(maxLineEditWidth, maxLineEditHeight);
//        presentPosLineEdits[i]->setMaximumWidth(maxLineEditWidth);
//        goalPosLineEdits[i]->setMaximumWidth(maxLineEditWidth);
//        presentSpeedLineEdits[i]->setMaximumWidth(maxLineEditWidth);
//        movingSpeedLineEdits[i]->setMaximumWidth(maxLineEditWidth);

        motorIdLabels[i]->setAlignment(Qt::AlignCenter);
        presentPosLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        goalPosLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        presentSpeedLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        movingSpeedLineEdits[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        presentPosLineEdits[i]->setReadOnly(true);
        goalPosLineEdits[i]->setReadOnly(true);
        presentSpeedLineEdits[i]->setReadOnly(true);
        movingSpeedLineEdits[i]->setReadOnly(true);

        row = i + 1;
        col = 0;
        motorFeedbackLayout->addWidget(motorIdLabels[i], row, col++);
        ++col;  // For vline
        motorFeedbackLayout->addWidget(presentPosSliders[i], row, col++);
        motorFeedbackLayout->addWidget(presentPosLineEdits[i], row, col++);
        ++col;  // For vline
        motorFeedbackLayout->addWidget(goalPosLineEdits[i], row, col++);
        ++col;  // For vline
        motorFeedbackLayout->addWidget(presentSpeedLineEdits[i], row, col++);
        ++col;  // For vline
        motorFeedbackLayout->addWidget(movingSpeedLineEdits[i], row, col++);
    }
    motorFeedbackLayout->setAlignment(Qt::AlignTop);
    motorFeedbackLayout->setColumnStretch(0, 0);
    motorFeedbackLayout->setColumnStretch(1, 2);
    motorFeedbackLayout->setColumnStretch(2, 1);
    motorFeedbackLayout->setColumnStretch(3, 1);
    motorFeedbackLayout->setColumnStretch(4, 1);
    // Vertical spacers
    QVector<QFrame*> vlineFrames(4);
    for (int i = 0; i < vlineFrames.size(); ++i)
    {
        vlineFrames[i] = new QFrame;
        vlineFrames[i]->setFrameShape(QFrame::VLine);
    }
    motorFeedbackLayout->addWidget(vlineFrames[3], 0, 8, motorFeedbackLayout->rowCount(), 1);
    motorFeedbackLayout->addWidget(vlineFrames[2], 0, 6, motorFeedbackLayout->rowCount(), 1);
    motorFeedbackLayout->addWidget(vlineFrames[1], 0, 4, motorFeedbackLayout->rowCount(), 1);
    motorFeedbackLayout->addWidget(vlineFrames[0], 0, 1, motorFeedbackLayout->rowCount(), 1);
    setLayout(motorFeedbackLayout);

    setMinimumWidth(500);
}


void MotorFeedbackWidget::updateJointStateValues(sensor_msgs::JointState js)
{
    if ( (js.position.size() >= NUM_OF_MOTORS) && (js.velocity.size() >= NUM_OF_MOTORS) )
    {
        for (int i = 0; i < NUM_OF_MOTORS; ++i)
        {
            // -2.56..2.555 rad, converted to -2560..2555 int range
            presentPosSliders[i]->setFirstValue(js.position[i] * 1000);

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
        }
    }
}


void MotorFeedbackWidget::updateSecondaryRobotValues(sensor_msgs::JointState js)
{
    if ( (js.position.size() >= NUM_OF_MOTORS) && (js.velocity.size() >= NUM_OF_MOTORS) )
    {
        for (int i = 0; i < NUM_OF_MOTORS; ++i)
        {
            std::ostringstream oss;
            oss.precision(3);
            oss.width(7);
            oss.fill(' ');
            oss.setf(std::ios::fixed, std::ios::floatfield);
            oss << js.position[i];
            goalPosLineEdits[i]->setText( QString::fromStdString(oss.str()) );

            oss.str("");
            oss.width(7);  // Not 'sticky'
            oss << js.velocity[i];
            movingSpeedLineEdits[i]->setText( QString::fromStdString(oss.str()) );
        }
    }
}


void MotorFeedbackWidget::updateJointStateValuesFromPose(sensor_msgs::JointState js)
{
    if (js.position.size() >= NUM_OF_MOTORS)
    {
    for (int i = 0; i < NUM_OF_MOTORS; ++i)
        presentPosSliders[i]->setSecondValue(js.position[i] * 1000);
    }
}
