#ifndef PIDBALANCERWIDGET_H
#define PIDBALANCERWIDGET_H

#include <qt5/QtWidgets/QWidget>
#include "rosworker.h"

class PidBalancerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PidBalancerWidget(RosWorker* rosWorker, QWidget* parent = 0);

signals:

public slots:

private:
    RosWorker* mRosWorker;
};

#endif // PIDBALANCERWIDGET_H
