#include "robotposeslistmodel.h"
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QIODevice>
#include <qt5/QtCore/QSet>
#include <qt5/QtCore/QChar>
#include <qt5/QtCore/QFile>
#include <qt5/QtCore/QTextStream>
#include <qt5/QtCore/QStringList>


RobotPosesListModel::RobotPosesListModel(QList<RobotPose> robotPosesList, bool allowDuplNames,
                                         QObject* parent) :
    QAbstractListModel(parent),
    mAllowDuplNames(allowDuplNames)
{
    if (!mAllowDuplNames)
    {
        // Don't add duplicates
        for (int i = 0; i < robotPosesList.size(); ++i)
        {
            bool duplName = false;
            for (int j = 0; j < mRobotPosesList.size(); ++j)
                if (mRobotPosesList[j].name == robotPosesList[i].name)
                    duplName = true;
            if (!duplName)
                mRobotPosesList.append(robotPosesList[i]);
        }
    }
    else
        mRobotPosesList = robotPosesList;
}


int RobotPosesListModel::rowCount(const QModelIndex &parent) const
{
    return mRobotPosesList.size();
}


QVariant RobotPosesListModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    if (index.row() >= mRobotPosesList.size())
        return QVariant();

    if (role == Qt::DisplayRole)
        return mRobotPosesList[index.row()].name;
    else
        return QVariant();
}


int RobotPosesListModel::add(RobotPose robotPose, QModelIndex &index)
{
    if (robotPose.name == "")
        return 1;

    // Check if duplicate
    if (!mAllowDuplNames)
        for (int i = 0; i < mRobotPosesList.size(); ++i)
            if (mRobotPosesList[i].name == robotPose.name)
                return 2;

    mRobotPosesList.insert(index.row(), robotPose);
    QModelIndex endIndex = QAbstractListModel::index(mRobotPosesList.size() - 1);
    emit dataChanged(index, endIndex);

    if (!index.isValid())
        index = QAbstractListModel::index(0);

    return 0;
}


void RobotPosesListModel::remove(QModelIndex &index)
{
    if (!index.isValid())
        return;

    mRobotPosesList.removeAt(index.row());
    QModelIndex endIndex = QAbstractListModel::index(mRobotPosesList.size() - 1);
    emit dataChanged(index, endIndex);
    if (index.row() > endIndex.row())
        index = QModelIndex();
}


void RobotPosesListModel::moveUp(QModelIndex &index)
{
    if (!index.isValid())
        return;

    //int position = currIndex.row();
    //beginRemoveRows(QModelIndex(), position, position+1);
    //mRobotPosesList.removeAt(position);
    //endRemoveRows();

    if (index.row() > 0)
    {
        QModelIndex prevIndex = QAbstractListModel::index(index.row() - 1);
        mRobotPosesList.swap(prevIndex.row(), index.row());
        emit dataChanged(prevIndex, index);
        //emit layoutChanged();

        index = prevIndex;
    }
}


void RobotPosesListModel::moveDown(QModelIndex &index)
{
    if (!index.isValid())
        return;

    if (index.row() < (mRobotPosesList.size() - 1))
    {
        QModelIndex nextIndex = QAbstractListModel::index(index.row() + 1);
        mRobotPosesList.swap(index.row(), nextIndex.row());
        emit dataChanged(index, nextIndex);

        index = nextIndex;
    }
}


void RobotPosesListModel::savePosesFile(QString fileName)
{
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream out(&file);
    out << QString("Pose name,").rightJustified(21);  // A name longer than 20 chars will mess up alignment
    out << QString("Dwell t,").rightJustified(11);
    for (int dxlID = 1; dxlID <= NUM_OF_MOTORS; ++dxlID)
    {
        out << QString("ID%1 pos,").arg(dxlID, 2, 10, QChar('0')).rightJustified(11);
        out << QString("ID%1 vel,").arg(dxlID, 2, 10, QChar('0')).rightJustified(11);

        if (dxlID < NUM_OF_MOTORS)
            out << QString("ID%1 eff,").arg(dxlID, 2, 10, QChar('0')).rightJustified(11);
        else
            out << QString("ID%1 eff\n").arg(dxlID, 2, 10, QChar('0')).rightJustified(11);
    }

    std::ostringstream oss;
    oss.precision(3);
    oss.fill(' ');
    oss.setf(std::ios::fixed, std::ios::floatfield);
    for (int i = 0; i < mRobotPosesList.size(); ++i)
    {
        if ( (mRobotPosesList[i].jointState.position.size() >= NUM_OF_MOTORS) &&
             (mRobotPosesList[i].jointState.velocity.size() >= NUM_OF_MOTORS) &&
             (mRobotPosesList[i].jointState.effort.size() >= NUM_OF_MOTORS) )
        {
            out << (mRobotPosesList[i].name + ",").rightJustified(21);

            oss.str("");
            oss.width(10);  // Not 'sticky'
            oss << mRobotPosesList[i].dwellTimeInSec;
            oss << ",";
            for (int j = 0; j < mRobotPosesList[i].jointState.position.size(); ++j)
            {
                // Position
                oss.width(10);
                oss << mRobotPosesList[i].jointState.position[j];
                oss << ",";

                // Velocity
                oss.width(10);
                oss << mRobotPosesList[i].jointState.velocity[j];
                oss << ",";

                // Effort
                oss.width(10);
                oss << mRobotPosesList[i].jointState.effort[j];

                if (j < (mRobotPosesList[i].jointState.position.size() - 1))
                    oss << ",";
            }

            out << QString::fromStdString(oss.str());
            out << "\n";
        }
    }

    file.close();
}


void RobotPosesListModel::loadPosesFile(QString fileName, QModelIndex &index)
{
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    mRobotPosesList.clear();

    QTextStream in(&file);
    in.readLine();  // Skip headers
    while (!in.atEnd())
    {
        QString line = in.readLine();
        QStringList field = line.split(",", QString::SkipEmptyParts);
        if ( field.size() >= (2 + NUM_OF_MOTORS*3) )  // Name, time and per-motor position/velocity/effort
        {
            RobotPose robotPose;
            robotPose.name = field[0];
            robotPose.dwellTimeInSec = field[1].toDouble();
            for (int j = 0; j < NUM_OF_MOTORS; ++j)
            {
                robotPose.jointState.position[j] = field[j*3 + 2].toDouble();
                robotPose.jointState.velocity[j] = field[j*3 + 3].toDouble();
                robotPose.jointState.effort  [j] = field[j*3 + 4].toDouble();
            }
            mRobotPosesList.append(robotPose);
        }
    }

    file.close();

    QModelIndex startIndex = QAbstractListModel::index(0);
    QModelIndex endIndex = QAbstractListModel::index(mRobotPosesList.size() - 1);
    emit dataChanged(startIndex, endIndex);
    index = startIndex;
}


RobotPose RobotPosesListModel::getCurrentPose(const QModelIndex &index) const
{
    if (!index.isValid())
        return RobotPose();
    else
        return mRobotPosesList.at(index.row());

}
