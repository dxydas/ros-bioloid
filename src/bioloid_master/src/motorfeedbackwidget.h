#ifndef MOTORFEEDBACKWIDGET_H
#define MOTORFEEDBACKWIDGET_H

#include <qt5/QtWidgets/QWidget>
#include <qt5/QtWidgets/QLineEdit>
#include "sensor_msgs/JointState.h"
#include "doubleslider.h"

class MotorFeedbackWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MotorFeedbackWidget(QWidget* parent = 0);

signals:

public slots:
    void updateJointStateValues(sensor_msgs::JointState js);
    void updateSecondaryRobotValues(sensor_msgs::JointState js);
    void updateJointStateValuesFromPose(sensor_msgs::JointState js);

private:
    QVector<DoubleSlider*> presentPosSliders;
    QVector<QLineEdit*> presentPosLineEdits;
    QVector<QLineEdit*> goalPosLineEdits;
    QVector<QLineEdit*> presentSpeedLineEdits;
    QVector<QLineEdit*> movingSpeedLineEdits;
};

#endif // MOTORFEEDBACKWIDGET_H
