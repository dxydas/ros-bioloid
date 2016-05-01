#include "motorcommandswidget.h"
#include <qt5/QtCore/QFile>
#include <qt5/QtCore/QString>
#include <qt5/QtWidgets/QSizePolicy>
#include <qt5/QtWidgets/QGridLayout>


MotorCommandsWidget::MotorCommandsWidget(QWidget* parent) :
    QWidget(parent)
{
    homeAllMotorsButton = new QPushButton("Home all motors");
    setAllMotorTorquesOffButton = new QPushButton("Set all motor torques OFF");
    setAllMotorTorquesOffButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QGridLayout *motorCommandsSubLayout = new QGridLayout;
    int row = 0;
    int col = 0;
    motorCommandsSubLayout->addWidget(homeAllMotorsButton, row, col++);
    motorCommandsSubLayout->addWidget(setAllMotorTorquesOffButton, row++, col);

//    QWidget* motorCommandsWidget =  new QWidget(this);
//    motorCommandsWidget->setLayout(motorCommandsSubLayout);
    setLayout(motorCommandsSubLayout);

    customiseLayout();
}


void MotorCommandsWidget::customiseLayout()
{
    QFile file;
    QString redButtonStyleSheet;

    file.setFileName("assets/qss/customredpushbutton.qss");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    redButtonStyleSheet.append( QLatin1String(file.readAll()) );
    file.close();

    setAllMotorTorquesOffButton->setStyleSheet(redButtonStyleSheet);
}
