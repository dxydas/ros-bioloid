#include "doubleslider.h"
#include <qt5/QtCore/QPoint>
#include <qt5/QtGui/QPainter>
#include <qt5/QtGui/QColor>
#include <qt5/QtWidgets/QAbstractSlider>
#include <qt5/QtWidgets/QStyle>
#include <qt5/QtWidgets/QStyleOptionSlider>
#include <QApplication>


DoubleSlider::DoubleSlider(QWidget* parent) :
    QSlider(parent), firstSliderValue(0), secondSliderValue(0)
{
}


DoubleSlider::DoubleSlider(Qt::Orientation orientation, QWidget* parent) :
    QSlider(orientation, parent)
{
}


void DoubleSlider::paintEvent(QPaintEvent* ev)
{
    QPainter* painter = new QPainter(this);
    QStyle* style = QApplication::style();
    QStyleOptionSlider* option = new QStyleOptionSlider();
    int min = QStyle::sliderPositionFromValue(minimum(), maximum(), minimum(), size().width());
    int max = QStyle::sliderPositionFromValue(minimum(), maximum(), maximum(), size().width());
    option->rect.setTopLeft(QPoint(min, 0.0));
    option->rect.setBottomRight(QPoint(max, size().height()));
    option->minimum = min;
    option->maximum = max;

    painter->setPen(Qt::blue);
    painter->setBrush(QColor("#66CCFF"));
    option->sliderPosition = QStyle::sliderPositionFromValue(minimum(), maximum(), secondSliderValue, size().width());;
    QRect rect = style->subControlRect(QStyle::CC_Slider, option, QStyle::SC_SliderHandle, this);
    painter->drawRect(rect);

    setStyleSheet( "QSlider::handle:horizontal {"
              "background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #B4B4B4, stop:1 #8F8F8F);"
              "border: 1px solid #5C5C5C;"
              "width: 18px;"
              "margin: -2px 0;"
              "border-radius: 3px;}" );
    option->sliderPosition = QStyle::sliderPositionFromValue(minimum(), maximum(), firstSliderValue, size().width());;
    style->drawComplexControl(QStyle::CC_Slider, option, painter, this);

    painter->end();
}

