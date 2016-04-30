#include "motorcommandswidget.h"
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
    QString buttonStyleSheet =
            "QPushButton {"
            "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
            "stop: 0 lightsteelblue, stop: 1 steelblue);"
            "border-color: #8F8F91;"
            "border-style: outset;"
            "border-width: 4px;"
            "border-radius: 10px; }"
            //"border-color: beige; }"
            //"font: bold 14px; }"
            //"min-width: 10em;"
            //"padding: 6px; }"
            "QPushButton:flat {"
            "border: none;"  /* no border for a flat push button */
            "}"
            "QPushButton:default {"
            "border-color: navy;"  /* make the default button prominent */
            "}"
            "QPushButton:pressed {"
            "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
            "stop: 0 royalblue, stop: 1 dodgerblue);"
            "border-style: inset; }";

    QString redButtonStyleSheet =
            "QPushButton {"
            "color: white;"
            "background-color: red;"
            "border: solid white;"
            "border-style: outset;"
            "border-width: 4px;"
            "border-radius: 4px; }"
            "QPushButton:flat {"
            "border: none;"  /* no border for a flat push button */
            "}"
            "QPushButton:default {"
            "border-color: grey;"  /* make the default button prominent */
            "}"
            "QPushButton:pressed {"
            "background-color: maroon;"
            "border-style: inset; }";

    homeAllMotorsButton->setStyleSheet(buttonStyleSheet);
    setAllMotorTorquesOffButton->setStyleSheet(redButtonStyleSheet);
}
