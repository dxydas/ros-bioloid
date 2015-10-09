#ifndef MOTORDIALS_H
#define MOTORDIALS_H

#include <QWidget>
#include <qt5/QtCore/QVector>
#include <qt5/QtCore/QTime>
#include <qt5/QtWidgets/QDial>
#include <qt5/QtWidgets/QLineEdit>
#include "rosworker.h"

class MotorDials : public QWidget
{
    Q_OBJECT
public:
    explicit MotorDials(RosWorker* rosWorker, QWidget* parent = 0);

signals:

public slots:
    void setMotor(int dxlId);
    void setValue(int value);
    void updateJointStateValues(sensor_msgs::JointState js);
    void initialiseDials(bool visible);

private:
    void customiseLayout();

    QVector<QDial*> dials;
    QVector<QLineEdit*> presentPosLineEdits;
    QVector<QLineEdit*> presentSpeedLineEdits;
    RosWorker* mRosWorker;
    int mSelectedMotor;
    QTime callTime;
};

#endif // MOTORDIALS_H
