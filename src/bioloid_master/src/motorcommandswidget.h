#ifndef MOTORCOMMANDSWIDGET_H
#define MOTORCOMMANDSWIDGET_H

#include <qt5/QtWidgets/QWidget>
#include <qt5/QtWidgets/QPushButton>

class MotorCommandsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MotorCommandsWidget(QWidget* parent = 0);
    QPushButton* homeAllMotorsButton;
    QPushButton* setAllMotorTorquesOffButton;

signals:

public slots:

private:
};

#endif // MOTORCOMMANDSWIDGET_H
