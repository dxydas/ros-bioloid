#ifndef DOUBLESLIDER_H
#define DOUBLESLIDER_H

#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QEvent>
#include <qt5/QtWidgets/QSlider>
#include <qt5/QtWidgets/QWidget>

class DoubleSlider : public QSlider
{
    Q_OBJECT
public:
    explicit DoubleSlider(QWidget* parent = 0);
    explicit DoubleSlider(Qt::Orientation orientation, QWidget* parent = 0);
    int getFirstSliderValue() const { return firstSliderValue; }
    int getSecondSliderValue() const { return secondSliderValue; }

signals:

public slots:
    void setValue(int value) { firstSliderValue = value; }
    void setSecondValue(int value) { secondSliderValue = value; }

private:
    //bool event(QEvent *event);
    void paintEvent(QPaintEvent* ev);
    int firstSliderValue;
    int secondSliderValue;
};

#endif // DOUBLESLIDER_H
