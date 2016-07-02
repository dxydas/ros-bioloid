#ifndef ROBOTPOSESLISTMODEL_H
#define ROBOTPOSESLISTMODEL_H

#include <qt5/QtCore/QAbstractListModel>
#include <qt5/QtCore/QObject>
#include <qt5/QtCore/QList>
#include <qt5/QtCore/QString>
#include "commonvars.h"

class RobotPosesListModel : public QAbstractListModel
{
    Q_OBJECT

public:
    explicit RobotPosesListModel(QList<RobotPose> robotPosesList, bool allowDuplNames = true,
                                 QObject* parent = 0);
    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    int add(RobotPose robotPose, QModelIndex &index);
    void remove(QModelIndex &index);
    void moveUp(QModelIndex &index);
    void moveDown(QModelIndex &index);
    void savePosesFile(QString fileName);
    void loadPosesFile(QString fileName, QModelIndex &index);

    QList<RobotPose> getRobotPosesList() const { return mRobotPosesList; }
    //void setRobotPosesList(const QList<RobotPose> &value) { mRobotPosesList = value; }
    RobotPose getCurrentPose(const QModelIndex &index) const;

signals:

public slots:

private:
    bool mAllowDuplNames;
    QList<RobotPose> mRobotPosesList;
};

#endif // ROBOTPOSESLISTMODEL_H
