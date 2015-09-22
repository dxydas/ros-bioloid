#include "robotposeslistmodel.h"
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QIODevice>
#include <qt5/QtCore/QSet>
#include <qt5/QtCore/QChar>
#include <qt5/QtCore/QFile>
#include <qt5/QtCore/QTextStream>
#include <qt5/QtCore/QStringList>


RobotPosesListModel::RobotPosesListModel(QList<RobotPoseStruct> robotPosesList, bool allowDuplNames,
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


int RobotPosesListModel::add(RobotPoseStruct robotPoseStruct, QModelIndex &index)
{
    if (robotPoseStruct.name == "")
        return 1;

    // Check if duplicate
    if (!mAllowDuplNames)
        for (int i = 0; i < mRobotPosesList.size(); ++i)
            if (mRobotPosesList[i].name == robotPoseStruct.name)
                return 2;

    mRobotPosesList.insert(index.row(), robotPoseStruct);
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
    out << QString("Pose name,").leftJustified(20);
    for (int dxlID = 1; dxlID <= NUM_OF_MOTORS; ++dxlID)
    {
        if (dxlID < NUM_OF_MOTORS)
            out << QString("ID%1,").arg(dxlID, 2, 10, QChar('0')).leftJustified(10);
        else
            out << "ID" << QString("%1").arg(dxlID, 2, 10, QChar('0'));
    }
    out << "\n";

    std::ostringstream oss;
    oss.precision(4);
    oss.fill('0');
    oss.setf(std::ios::fixed, std::ios::floatfield);
    for (int i = 0; i < mRobotPosesList.size(); ++i)
    {
        if ( mRobotPosesList[i].jointState.position.size() == NUM_OF_MOTORS )
        {
            out << (mRobotPosesList[i].name + ",").leftJustified(20);

            oss.str("");
            for (int j = 0; j < mRobotPosesList[i].jointState.position.size(); ++j)
            {
                oss.width(7);  // Not 'sticky'
                oss << mRobotPosesList[i].jointState.position[j];
                if (j < (mRobotPosesList[i].jointState.position.size() - 1))
                    oss << ",  ";
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
    while (!in.atEnd())
    {
        QString line = in.readLine();
        QStringList field = line.split(",");
        if ( field.size() == (NUM_OF_MOTORS + 1) )
        {
            RobotPoseStruct poseStruct;
            poseStruct.name = field[0];
            poseStruct.jointState.position.resize(NUM_OF_MOTORS);
            for (int i = 0; i < NUM_OF_MOTORS; ++i)
                poseStruct.jointState.position[i] = field[i+1].toDouble();
            mRobotPosesList.append(poseStruct);
        }
    }

    file.close();

    QModelIndex startIndex = QAbstractListModel::index(0);
    QModelIndex endIndex = QAbstractListModel::index(mRobotPosesList.size() - 1);
    emit dataChanged(startIndex, endIndex);
    index = startIndex;
}


RobotPoseStruct RobotPosesListModel::getCurrentPose(const QModelIndex &index) const
{
    if (!index.isValid())
        return RobotPoseStruct();
    else
        return mRobotPosesList.at(index.row());

}

