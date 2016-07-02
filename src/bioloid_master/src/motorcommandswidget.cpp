#include "motorcommandswidget.h"
#include <qt5/QtCore/QObject>
#include <qt5/QtWidgets/QAbstractButton>
#include <qt5/QtWidgets/QSizePolicy>
#include <qt5/QtWidgets/QGridLayout>


MotorCommandsWidget::MotorCommandsWidget(QWidget* parent) :
    QWidget(parent)
{
    homeAllMotorsButton = new QPushButton("Home all motors");
    homeAllMotorsButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    setAllMotorTorquesOffButton = new QPushButton("Set all motor torques OFF");
    setAllMotorTorquesOffButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setAllMotorTorquesOffButton->setObjectName("redPushButton");

    QGridLayout *motorCommandsSubLayout = new QGridLayout;
    int row = 0;
    int col = 0;
    motorCommandsSubLayout->addWidget(homeAllMotorsButton, row, col++);
    motorCommandsSubLayout->addWidget(setAllMotorTorquesOffButton, row++, col);

//    QWidget* motorCommandsWidget =  new QWidget(this);
//    motorCommandsWidget->setLayout(motorCommandsSubLayout);
    setLayout(motorCommandsSubLayout);
}
