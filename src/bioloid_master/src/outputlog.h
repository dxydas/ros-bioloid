#ifndef OUTPUTLOG_H
#define OUTPUTLOG_H

#include <qt5/QtWidgets/QTextEdit>

class OutputLog : public QTextEdit
{
    Q_OBJECT
public:
    explicit OutputLog(QWidget *parent = 0);

private:

signals:

public slots:
    void appendTimestamped(const QString & text);

};

#endif // OUTPUTLOG_H
