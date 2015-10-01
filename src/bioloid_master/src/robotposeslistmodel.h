#ifndef ROBOTPOSESLISTMODEL_H
#define ROBOTPOSESLISTMODEL_H

#include <qt5/QtCore/QAbstractListModel>
#include <qt5/QtCore/QObject>
#include <qt5/QtCore/QList>
#include <qt5/QtCore/QString>
#include <moveit/move_group_interface/move_group.h>

#define NUM_OF_MOTORS 18

struct RobotPoseStruct
{
    QString name;
    //geometry_msgs::Pose pose;
    //QVector<double> jointPositions;
    sensor_msgs::JointState jointState;
    RobotPoseStruct()
    {
        name = "";
        //QVector<double> jp(NUM_OF_MOTORS, 0.0);
        //jointPositions = jp;
        jointState.position.resize(NUM_OF_MOTORS + 1);
        jointState.velocity.resize(NUM_OF_MOTORS + 1);
    }
};

class RobotPosesListModel : public QAbstractListModel
{
    Q_OBJECT

public:
    explicit RobotPosesListModel(QList<RobotPoseStruct> robotPosesList, bool allowDuplNames = true,
                                 QObject* parent = 0);
    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    int add(RobotPoseStruct robotPoseStruct, QModelIndex &index);
    void remove(QModelIndex &index);
    void moveUp(QModelIndex &index);
    void moveDown(QModelIndex &index);
    void savePosesFile(QString fileName);
    void loadPosesFile(QString fileName, QModelIndex &index);

    QList<RobotPoseStruct> getRobotPosesList() const { return mRobotPosesList; }
    //void setRobotPosesList(const QList<RobotPoseStruct> &value) { mRobotPosesList = value; }
    RobotPoseStruct getCurrentPose(const QModelIndex &index) const;

private:
    bool mAllowDuplNames;
    QList<RobotPoseStruct> mRobotPosesList;

signals:

public slots:

};

#endif // ROBOTPOSESLISTMODEL_H
