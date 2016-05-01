#include "motorfeedbackwidget.h"
#include <iostream>
#include <sstream>
#include <qt5/QtCore/QString>
#include <qt5/QtCore/QVector>
#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QFrame>
#include "commonvars.h"


MotorFeedbackWidget::MotorFeedbackWidget(QWidget* parent) :
    QWidget(parent)
{
    QLabel* motorLabel = new QLabel("Motor");
    presentPositionAndSelectedPoseLabel = new QLabel("Present position\nand selected pose");
    presentPositionLabel = new QLabel("Present\nposition");
    goalPositionLabel = new QLabel("Goal\nposition");
    presentSpeedLabel = new QLabel("Present\nspeed");
    movingSpeedLabel = new QLabel("Moving\nspeed");

    motorLabel->setAlignment(Qt::AlignCenter);
    presentPositionAndSelectedPoseLabel->setAlignment(Qt::AlignCenter);
    presentPositionLabel->setAlignment(Qt::AlignCenter);
    goalPositionLabel->setAlignment(Qt::AlignCenter);
    presentSpeedLabel->setAlignment(Qt::AlignCenter);
    movingSpeedLabel->setAlignment(Qt::AlignCenter);

    QGridLayout* motorFeedbackSubLayout = new QGridLayout;
    int row = 0;
    int col = 0;
    motorFeedbackSubLayout->addWidget(motorLabel, row, col++);
    ++col;  // Leave a blank column in order to add vline spacer later
    motorFeedbackSubLayout->addWidget(presentPositionAndSelectedPoseLabel, row, col++);
    motorFeedbackSubLayout->addWidget(presentPositionLabel, row, col++);
    ++col;  // For vline
    motorFeedbackSubLayout->addWidget(goalPositionLabel, row, col++);
    ++col;  // For vline
    motorFeedbackSubLayout->addWidget(presentSpeedLabel, row, col++);
    ++col;  // For vline
    motorFeedbackSubLayout->addWidget(movingSpeedLabel, row, col++);

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
        presentPosSliders[i]->setMinimum(-2560);
        presentPosSliders[i]->setMaximum(2555);
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

        row = i + 1;
        col = 0;
        motorFeedbackSubLayout->addWidget(motorIdLabels[i], row, col++);
        ++col;  // For vline
        motorFeedbackSubLayout->addWidget(presentPosSliders[i], row, col++);
        motorFeedbackSubLayout->addWidget(presentPosLineEdits[i], row, col++);
        ++col;  // For vline
        motorFeedbackSubLayout->addWidget(goalPosLineEdits[i], row, col++);
        ++col;  // For vline
        motorFeedbackSubLayout->addWidget(presentSpeedLineEdits[i], row, col++);
        ++col;  // For vline
        motorFeedbackSubLayout->addWidget(movingSpeedLineEdits[i], row, col++);
    }
    motorFeedbackSubLayout->setAlignment(Qt::AlignTop);
    motorFeedbackSubLayout->setColumnStretch(0, 0);
    motorFeedbackSubLayout->setColumnStretch(1, 2);
    motorFeedbackSubLayout->setColumnStretch(2, 1);
    motorFeedbackSubLayout->setColumnStretch(3, 1);
    motorFeedbackSubLayout->setColumnStretch(4, 1);
    // Vertical spacers
    QVector<QFrame*> vlineFrames(4);
    for (int i = 0; i < vlineFrames.size(); ++i)
    {
        vlineFrames[i] = new QFrame;
        vlineFrames[i]->setFrameShape(QFrame::VLine);
    }
    motorFeedbackSubLayout->addWidget(vlineFrames[3], 0, 8, motorFeedbackSubLayout->rowCount(), 1);
    motorFeedbackSubLayout->addWidget(vlineFrames[2], 0, 6, motorFeedbackSubLayout->rowCount(), 1);
    motorFeedbackSubLayout->addWidget(vlineFrames[1], 0, 4, motorFeedbackSubLayout->rowCount(), 1);
    motorFeedbackSubLayout->addWidget(vlineFrames[0], 0, 1, motorFeedbackSubLayout->rowCount(), 1);

    customiseLayout();

    //QWidget* motorFeedbackWidget = new QWidget(this);
    //motorFeedbackWidget->setLayout(motorFeedbackSubLayout);
    setLayout(motorFeedbackSubLayout);
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


void MotorFeedbackWidget::customiseLayout()
{
    presentPositionAndSelectedPoseLabel->setStyleSheet("QLabel { color: royalblue }");
    presentPositionLabel->setStyleSheet("QLabel { color: royalblue }");
    goalPositionLabel->setStyleSheet("QLabel { color: midnightblue }");
    presentSpeedLabel->setStyleSheet("QLabel { color: royalblue }");
    movingSpeedLabel->setStyleSheet("QLabel { color: midnightblue }");
}
