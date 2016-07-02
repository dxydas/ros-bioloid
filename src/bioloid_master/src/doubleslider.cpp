#include "doubleslider.h"
#include <qt5/QtCore/QPoint>
#include <qt5/QtCore/QRect>
#include <qt5/QtGui/QPainter>
#include <qt5/QtGui/QColor>
#include <qt5/QtWidgets/QStyle>
#include <qt5/QtWidgets/QStyleOptionSlider>
#include <qt5/QtWidgets/QApplication>


DoubleSlider::DoubleSlider(Qt::Orientation orientation, QWidget* parent) :
    QSlider(orientation, parent), firstSliderValue(0), secondSliderValue(0)
{
    connect( this, SIGNAL(firstValueChanged(int)), this, SLOT(update()) );
    connect( this, SIGNAL(secondValueChanged(int)), this, SLOT(update()) );
}


void DoubleSlider::setFirstValue(int value)
{
    firstSliderValue = value;
    emit firstValueChanged(value);
}


void DoubleSlider::setSecondValue(int value)
{
    secondSliderValue = value;
    emit secondValueChanged(value);
}


void DoubleSlider::paintEvent(QPaintEvent* ev)
{
    QPainter painter(this);
    QStyleOptionSlider* option = new QStyleOptionSlider;
    int min = QStyle::sliderPositionFromValue(minimum(), maximum(), minimum(), size().width());
    int max = QStyle::sliderPositionFromValue(minimum(), maximum(), maximum(), size().width());
    option->rect.setTopLeft(QPoint(min, 0.0));
    option->rect.setBottomRight(QPoint(max, size().height()));
    option->minimum = min;
    option->maximum = max;

    // Draw groove
    painter.setPen(QColor("steelblue"));
    painter.setBrush(QColor("lightsteelblue"));
    QRect rect = style()->subControlRect(QStyle::CC_Slider, option, QStyle::SC_SliderGroove, this);
    rect.adjust(0, 0, -2, -1);
    painter.drawRect(rect);

    // Draw second handle
    painter.setPen(QColor("black"));
    painter.setBrush(QColor("darkorange"));
    option->sliderPosition = style()->sliderPositionFromValue(minimum(), maximum(), secondSliderValue, size().width());;
    rect = style()->subControlRect(QStyle::CC_Slider, option, QStyle::SC_SliderHandle, this);
    rect.adjust(0, 0, -1, -1);
    painter.drawRect(rect);

    // Draw first handle
    painter.setPen(QColor("blue"));
    painter.setBrush(QColor("royalblue"));
    option->sliderPosition = style()->sliderPositionFromValue(minimum(), maximum(), firstSliderValue, size().width());;
    rect = style()->subControlRect(QStyle::CC_Slider, option, QStyle::SC_SliderHandle, this);
    rect.adjust(0, 0, -1, -1);
    painter.drawRect(rect);
}
