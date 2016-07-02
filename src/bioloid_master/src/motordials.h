#ifndef MOTORDIALS_H
#define MOTORDIALS_H

#include <qt5/QtWidgets/QWidget>
#include <qt5/QtCore/QVector>
#include <qt5/QtCore/QTime>
#include <qt5/QtCore/QTimer>
#include <qt5/QtWidgets/QGroupBox>
#include <qt5/QtWidgets/QDial>
#include <qt5/QtWidgets/QLineEdit>
#include <qt5/QtWidgets/QComboBox>
#include "rosworker.h"

class MotorDials : public QWidget
{
    Q_OBJECT

public:
    explicit MotorDials(RosWorker* rosWorker, QWidget* parent = 0);

signals:

public slots:
    void initialiseDials(bool visible);
    void setMotor(int dxlId);
    void setDialOption(int index);
    void setValue(int value);
    void updateJointStateValues(sensor_msgs::JointState js);
    void updateLineEdits();
    void toggleAutoRefresh(int toggle);

private:
    void customiseLayout();
    QVector<QGroupBox*> groupBoxes;
    QVector<QDial*> dials;
    QVector<QComboBox*> dialOptionComboBoxes;
    QVector<QLineEdit*> presentPosLineEdits;
    QVector<QLineEdit*> presentSpeedLineEdits;
    QVector<QLineEdit*> presentLoadLineEdits;
    QVector<QLineEdit*> goalValueLineEdits;
    QVector<QLineEdit*> presentVoltageLineEdits;
    QVector<QLineEdit*> presentTempLineEdits;
    QVector<QLineEdit*> torqueEnableLineEdits;
    QVector<QLineEdit*> ledLineEdits;
    QVector<QLineEdit*> alarmLedLineEdits;
    QVector< QVector<QLineEdit*> > alarmLedVectors;
    RosWorker* mRosWorker;
    int mSelectedMotor;
    QVector<int> mSelectedDialOption;
    QTime callTime;
    QTimer* feedbackTimer;
    bool dialsInitialised;
    QString ledOffLineEdit;
    QString ledOnLineEdit;
    QString ledArrayOffLineEdit;
    QString ledArrayOnLineEdit;
};

#endif // MOTORDIALS_H
