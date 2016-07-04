#include "outputlog.h"
#include <qt5/QtCore/QDateTime>


OutputLog::OutputLog(QWidget *parent) :
    QTextEdit(parent)
{
    setReadOnly(true);
}


void OutputLog::appendTimestamped(const QString & text)
{
    QDateTime currentTime = QDateTime::currentDateTime();
    append(currentTime.toString("ddd dd MMMM yyyy, hh:mm:ss ") + text);
}
