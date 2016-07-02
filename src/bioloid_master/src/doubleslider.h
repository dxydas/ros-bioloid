#ifndef DOUBLESLIDER_H
#define DOUBLESLIDER_H

#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QEvent>
#include <qt5/QtWidgets/QAbstractSlider>
#include <qt5/QtWidgets/QSlider>
#include <qt5/QtWidgets/QWidget>

class DoubleSlider : public QSlider
{
    Q_OBJECT

public:
    explicit DoubleSlider(Qt::Orientation orientation = Qt::Horizontal, QWidget* parent = 0);
    int getFirstSliderValue() const { return firstSliderValue; }
    int getSecondSliderValue() const { return secondSliderValue; }

signals:
    void firstValueChanged(int value);
    void secondValueChanged(int value);

public slots:
    void setFirstValue(int value);
    void setSecondValue(int value);

private:
    void paintEvent(QPaintEvent* ev);
    int firstSliderValue;
    int secondSliderValue;
};

#endif // DOUBLESLIDER_H
