#include "pidwidget.h"
#include <sstream>
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QObject>
#include <qt5/QtCore/QString>
#include <qt5/QtWidgets/QFormLayout>
#include <qt5/QtWidgets/QHBoxLayout>
#include <qt5/QtWidgets/QVBoxLayout>
#include <qt5/QtWidgets/QSizePolicy>


PidWidget::PidWidget(SimplePid *pid, QWidget* parent) :
    pid(pid), QWidget(parent)
{
    pGainMaxSpinBox = new QDoubleSpinBox();
    iGainMaxSpinBox = new QDoubleSpinBox();
    dGainMaxSpinBox = new QDoubleSpinBox();
    outMaxMaxSpinBox = new QDoubleSpinBox();
    outMinMaxSpinBox = new QDoubleSpinBox();
    pGainMinSpinBox = new QDoubleSpinBox();
    iGainMinSpinBox = new QDoubleSpinBox();
    dGainMinSpinBox = new QDoubleSpinBox();
    outMaxMinSpinBox = new QDoubleSpinBox();
    outMinMinSpinBox = new QDoubleSpinBox();

    pGainCurrentSpinBox = new QDoubleSpinBox;
    iGainCurrentSpinBox = new QDoubleSpinBox;
    dGainCurrentSpinBox = new QDoubleSpinBox;
    outMaxCurrentSpinBox = new QDoubleSpinBox;
    outMinCurrentSpinBox = new QDoubleSpinBox;

    pGainSlider = new QSlider(Qt::Vertical, parent);
    iGainSlider = new QSlider(Qt::Vertical, parent);
    dGainSlider = new QSlider(Qt::Vertical, parent);
    outMaxSlider = new QSlider(Qt::Vertical, parent);
    outMinSlider = new QSlider(Qt::Vertical, parent);

    pGainSlider->setObjectName("blueSlider");
    iGainSlider->setObjectName("blueSlider");
    dGainSlider->setObjectName("blueSlider");
    outMaxSlider->setObjectName("blueSlider");
    outMinSlider->setObjectName("blueSlider");

    QFormLayout* pGainMaxLayout = new QFormLayout;
    QFormLayout* iGainMaxLayout = new QFormLayout;
    QFormLayout* dGainMaxLayout = new QFormLayout;
    QFormLayout* outMaxMaxLayout = new QFormLayout;
    QFormLayout* outMinMaxLayout = new QFormLayout;
    QFormLayout* pGainMinLayout = new QFormLayout;
    QFormLayout* iGainMinLayout = new QFormLayout;
    QFormLayout* dGainMinLayout = new QFormLayout;
    QFormLayout* outMaxMinLayout = new QFormLayout;
    QFormLayout* outMinMinLayout = new QFormLayout;

    pGainMaxLayout->addRow("Max:", pGainMaxSpinBox);
    iGainMaxLayout->addRow("Max:", iGainMaxSpinBox);
    dGainMaxLayout->addRow("Max:", dGainMaxSpinBox);
    outMaxMaxLayout->addRow("Max:", outMaxMaxSpinBox);
    outMinMaxLayout->addRow("Max:", outMinMaxSpinBox);
    pGainMinLayout->addRow("Min:", pGainMinSpinBox);
    iGainMinLayout->addRow("Min:", iGainMinSpinBox);
    dGainMinLayout->addRow("Min:", dGainMinSpinBox);
    outMaxMinLayout->addRow("Min:", outMaxMinSpinBox);
    outMinMinLayout->addRow("Min:", outMinMinSpinBox);

    QHBoxLayout* pGainSliderLayout = new QHBoxLayout;
    pGainSliderLayout->addWidget(pGainCurrentSpinBox, 0, Qt::AlignRight);
    pGainSliderLayout->addWidget(pGainSlider, 0, Qt::AlignHCenter);

    QHBoxLayout* iGainSliderLayout = new QHBoxLayout;
    iGainSliderLayout->addWidget(iGainCurrentSpinBox, 0, Qt::AlignRight);
    iGainSliderLayout->addWidget(iGainSlider, 0, Qt::AlignHCenter);

    QHBoxLayout* dGainSliderLayout = new QHBoxLayout;
    dGainSliderLayout->addWidget(dGainCurrentSpinBox, 0, Qt::AlignRight);
    dGainSliderLayout->addWidget(dGainSlider, 0, Qt::AlignHCenter);

    QHBoxLayout* outMaxSliderLayout = new QHBoxLayout;
    outMaxSliderLayout->addWidget(outMaxCurrentSpinBox, 0, Qt::AlignRight);
    outMaxSliderLayout->addWidget(outMaxSlider, 0, Qt::AlignHCenter);

    QHBoxLayout* outMinSliderLayout = new QHBoxLayout;
    outMinSliderLayout->addWidget(outMinCurrentSpinBox, 0, Qt::AlignRight);
    outMinSliderLayout->addWidget(outMinSlider, 0, Qt::AlignHCenter);

    QVBoxLayout* pGainLayout = new QVBoxLayout;
    pGainLayout->addLayout(pGainMaxLayout);
    pGainLayout->addLayout(pGainSliderLayout);
    pGainLayout->addLayout(pGainMinLayout);

    QVBoxLayout* iGainLayout = new QVBoxLayout;
    iGainLayout->addLayout(iGainMaxLayout);
    iGainLayout->addLayout(iGainSliderLayout);
    iGainLayout->addLayout(iGainMinLayout);

    QVBoxLayout* dGainLayout = new QVBoxLayout;
    dGainLayout->addLayout(dGainMaxLayout);
    dGainLayout->addLayout(dGainSliderLayout);
    dGainLayout->addLayout(dGainMinLayout);

    QVBoxLayout* outMaxLayout = new QVBoxLayout;
    outMaxLayout->addLayout(outMaxMaxLayout);
    outMaxLayout->addLayout(outMaxSliderLayout);
    outMaxLayout->addLayout(outMaxMinLayout);

    QVBoxLayout* outMinLayout = new QVBoxLayout;
    outMinLayout->addLayout(outMinMaxLayout);
    outMinLayout->addLayout(outMinSliderLayout);
    outMinLayout->addLayout(outMinMinLayout);

    pGainGroupBox = new QGroupBox("P-Gain");
    pGainGroupBox->setLayout(pGainLayout);

    iGainGroupBox = new QGroupBox("I-Gain");
    iGainGroupBox->setLayout(iGainLayout);

    dGainGroupBox = new QGroupBox("D-Gain");
    dGainGroupBox->setLayout(dGainLayout);

    outMaxGroupBox = new QGroupBox("O/p Max");
    outMaxGroupBox->setLayout(outMaxLayout);
    outMaxGroupBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

    outMinGroupBox = new QGroupBox("O/p Min");
    outMinGroupBox->setLayout(outMinLayout);
    outMinGroupBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

    QHBoxLayout* outLayout = new QHBoxLayout;
    outLayout->addWidget(outMaxGroupBox);
    outLayout->addWidget(outMinGroupBox);

    QHBoxLayout* gainsLayout = new QHBoxLayout;
    gainsLayout->addWidget(pGainGroupBox);
    gainsLayout->addWidget(iGainGroupBox);
    gainsLayout->addWidget(dGainGroupBox);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(outLayout);
    mainLayout->addLayout(gainsLayout);
    setLayout(mainLayout);

    connect( pGainMaxSpinBox, SIGNAL(valueChanged(double)), this, SLOT(pGainMaxUpdated(double)) );
    connect( iGainMaxSpinBox, SIGNAL(valueChanged(double)), this, SLOT(iGainMaxUpdated(double)) );
    connect( dGainMaxSpinBox, SIGNAL(valueChanged(double)), this, SLOT(dGainMaxUpdated(double)) );
    connect( outMaxMaxSpinBox, SIGNAL(valueChanged(double)), this, SLOT(outMaxMaxUpdated(double)) );
    connect( outMinMaxSpinBox, SIGNAL(valueChanged(double)), this, SLOT(outMinMaxUpdated(double)) );
    connect( pGainMinSpinBox, SIGNAL(valueChanged(double)), this, SLOT(pGainMinUpdated(double)) );
    connect( iGainMinSpinBox, SIGNAL(valueChanged(double)), this, SLOT(iGainMinUpdated(double)) );
    connect( dGainMinSpinBox, SIGNAL(valueChanged(double)), this, SLOT(dGainMinUpdated(double)) );
    connect( outMaxMinSpinBox, SIGNAL(valueChanged(double)), this, SLOT(outMaxMinUpdated(double)) );
    connect( outMinMinSpinBox, SIGNAL(valueChanged(double)), this, SLOT(outMinMinUpdated(double)) );
    connect( pGainCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updatePGainFromSpinBox(double)) );
    connect( iGainCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateIGainFromSpinBox(double)) );
    connect( dGainCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateDGainFromSpinBox(double)) );
    connect( outMaxCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateOutMaxFromSpinBox(double)) );
    connect( outMinCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateOutMinFromSpinBox(double)) );
    connect( pGainSlider, SIGNAL(valueChanged(int)), this, SLOT(updatePGainFromSlider(int)) );
    connect( iGainSlider, SIGNAL(valueChanged(int)), this, SLOT(updateIGainFromSlider(int)) );
    connect( dGainSlider, SIGNAL(valueChanged(int)), this, SLOT(updateDGainFromSlider(int)) );
    connect( outMaxSlider, SIGNAL(valueChanged(int)), this, SLOT(updateOutMaxFromSlider(int)) );
    connect( outMinSlider, SIGNAL(valueChanged(int)), this, SLOT(updateOutMinFromSlider(int)) );

    pGainMaxSpinBox->setRange(-99.999, 99.999);
    pGainMaxSpinBox->setDecimals(3);
    pGainMaxSpinBox->setSingleStep(0.1);
    pGainMaxSpinBox->setValue(10.0);
    iGainMaxSpinBox->setRange(-99.999, 99.999);
    iGainMaxSpinBox->setDecimals(3);
    iGainMaxSpinBox->setSingleStep(0.1);
    iGainMaxSpinBox->setValue(10.0);
    dGainMaxSpinBox->setRange(-99.999, 99.999);
    dGainMaxSpinBox->setDecimals(3);
    dGainMaxSpinBox->setSingleStep(0.1);
    dGainMaxSpinBox->setValue(10.0);
    outMaxMaxSpinBox->setRange(-99.999, 99.999);
    outMaxMaxSpinBox->setDecimals(3);
    outMaxMaxSpinBox->setSingleStep(0.1);
    outMaxMaxSpinBox->setValue(50.0);
    outMinMaxSpinBox->setRange(-99.999, 99.999);
    outMinMaxSpinBox->setDecimals(3);
    outMinMaxSpinBox->setSingleStep(0.1);
    outMinMaxSpinBox->setValue(0.0);

    pGainMinSpinBox->setRange(-99.999, 99.999);
    pGainMinSpinBox->setDecimals(3);
    pGainMinSpinBox->setSingleStep(0.1);
    pGainMinSpinBox->setValue(-10.0);
    iGainMinSpinBox->setRange(-99.999, 99.999);
    iGainMinSpinBox->setDecimals(3);
    iGainMinSpinBox->setSingleStep(0.1);
    iGainMinSpinBox->setValue(-10.0);
    dGainMinSpinBox->setRange(-99.999, 99.999);
    dGainMinSpinBox->setDecimals(3);
    dGainMinSpinBox->setSingleStep(0.1);
    dGainMinSpinBox->setValue(-10.0);
    outMaxMinSpinBox->setRange(-99.999, 99.999);
    outMaxMinSpinBox->setDecimals(3);
    outMaxMinSpinBox->setSingleStep(0.1);
    outMaxMinSpinBox->setValue(0.0);
    outMinMinSpinBox->setRange(-99.999, 99.999);
    outMinMinSpinBox->setDecimals(3);
    outMinMinSpinBox->setSingleStep(0.1);
    outMinMinSpinBox->setValue(-50.0);

    double pGainCurrentValue = pid->getPGain();
    double iGainCurrentValue = pid->getIGain();
    double dGainCurrentValue = pid->getDGain();
    double outMaxCurrentValue = pid->getOutMax();
    double outMinCurrentValue = pid->getOutMin();

    pGainCurrentSpinBox->setValue(pGainCurrentValue);
    pGainCurrentSpinBox->setDecimals(3);
    pGainCurrentSpinBox->setSingleStep(0.1);

    iGainCurrentSpinBox->setValue(iGainCurrentValue);
    iGainCurrentSpinBox->setDecimals(3);
    iGainCurrentSpinBox->setSingleStep(0.1);

    dGainCurrentSpinBox->setValue(dGainCurrentValue);
    dGainCurrentSpinBox->setDecimals(3);
    dGainCurrentSpinBox->setSingleStep(0.1);

    outMaxCurrentSpinBox->setValue(outMaxCurrentValue);
    outMaxCurrentSpinBox->setDecimals(3);
    outMaxCurrentSpinBox->setSingleStep(0.1);

    outMinCurrentSpinBox->setValue(outMinCurrentValue);
    outMinCurrentSpinBox->setDecimals(3);
    outMinCurrentSpinBox->setSingleStep(0.1);
}


void PidWidget::pGainMaxUpdated(double value)
{
    if ( value < pGainMinSpinBox->value() )
        pGainMaxSpinBox->setValue(pGainMinSpinBox->value());
    double newValue = pGainMaxSpinBox->value();
    int intNewValue = newValue*1000;
    pGainCurrentSpinBox->setMaximum(newValue);
    pGainSlider->setMaximum(intNewValue);
}


void PidWidget::iGainMaxUpdated(double value)
{
    if ( value < iGainMinSpinBox->value() )
        iGainMaxSpinBox->setValue(iGainMinSpinBox->value());
    double newValue = iGainMaxSpinBox->value();
    int intNewValue = newValue*1000;
    iGainCurrentSpinBox->setMaximum(newValue);
    iGainSlider->setMaximum(intNewValue);
}


void PidWidget::dGainMaxUpdated(double value)
{
    if ( value < dGainMinSpinBox->value() )
        dGainMaxSpinBox->setValue(dGainMinSpinBox->value());
    double newValue = dGainMaxSpinBox->value();
    int intNewValue = newValue*1000;
    dGainCurrentSpinBox->setMaximum(newValue);
    dGainSlider->setMaximum(intNewValue);
}


void PidWidget::outMaxMaxUpdated(double value)
{
    if ( value < outMaxMinSpinBox->value() )
        outMaxMaxSpinBox->setValue(outMaxMinSpinBox->value());
    double newValue = outMaxMaxSpinBox->value();
    int intNewValue = newValue*1000;
    outMaxCurrentSpinBox->setMaximum(newValue);
    outMaxSlider->setMaximum(intNewValue);
}


void PidWidget::outMinMaxUpdated(double value)
{
    if ( value < outMinMinSpinBox->value() )
        outMinMaxSpinBox->setValue(outMinMinSpinBox->value());
    double newValue = outMinMaxSpinBox->value();
    int intNewValue = newValue*1000;
    outMinCurrentSpinBox->setMaximum(newValue);
    outMinSlider->setMaximum(intNewValue);
}


void PidWidget::pGainMinUpdated(double value)
{
    if ( value > pGainMaxSpinBox->value() )
        pGainMinSpinBox->setValue(pGainMaxSpinBox->value());
    double newValue = pGainMinSpinBox->value();
    int intNewValue = newValue*1000;
    pGainCurrentSpinBox->setMinimum(newValue);
    pGainSlider->setMinimum(intNewValue);
}


void PidWidget::iGainMinUpdated(double value)
{
    if ( value > iGainMaxSpinBox->value() )
        iGainMinSpinBox->setValue(iGainMaxSpinBox->value());
    double newValue = iGainMinSpinBox->value();
    int intNewValue = newValue*1000;
    iGainCurrentSpinBox->setMinimum(newValue);
    iGainSlider->setMinimum(intNewValue);
}


void PidWidget::dGainMinUpdated(double value)
{
    if ( value > dGainMaxSpinBox->value() )
        dGainMinSpinBox->setValue(dGainMaxSpinBox->value());
    double newValue = dGainMinSpinBox->value();
    int intNewValue = newValue*1000;
    dGainCurrentSpinBox->setMinimum(newValue);
    dGainSlider->setMinimum(intNewValue);
}


void PidWidget::outMaxMinUpdated(double value)
{
    if ( value > outMaxMaxSpinBox->value() )
        outMaxMinSpinBox->setValue(outMaxMaxSpinBox->value());
    double newValue = outMaxMinSpinBox->value();
    int intNewValue = newValue*1000;
    outMaxCurrentSpinBox->setMinimum(newValue);
    outMaxSlider->setMinimum(intNewValue);
}


void PidWidget::outMinMinUpdated(double value)
{
    if ( value > outMinMaxSpinBox->value() )
        outMinMinSpinBox->setValue(outMinMaxSpinBox->value());
    double newValue = outMinMinSpinBox->value();
    int intNewValue = newValue*1000;
    outMinCurrentSpinBox->setMinimum(newValue);
    outMinSlider->setMinimum(intNewValue);
}


void PidWidget::updatePGainFromSpinBox(double value)
{
    int intValue = value*1000;
    // Temporarily disconnect signal-slot to avoid recursion between spinbox and slider
    disconnect( pGainSlider, SIGNAL(valueChanged(int)), this, SLOT(updatePGainFromSlider(int)) );
    pGainSlider->setValue(intValue);
    connect( pGainSlider, SIGNAL(valueChanged(int)), this, SLOT(updatePGainFromSlider(int)) );
    pid->setPGain(value);
}


void PidWidget::updateIGainFromSpinBox(double value)
{
    int intValue = value*1000;
    // Temporarily disconnect signal-slot to avoid recursion between spinbox and slider
    disconnect( iGainSlider, SIGNAL(valueChanged(int)), this, SLOT(updateIGainFromSlider(int)) );
    iGainSlider->setValue(intValue);
    connect( iGainSlider, SIGNAL(valueChanged(int)), this, SLOT(updateIGainFromSlider(int)) );
    pid->setIGain(value);
}


void PidWidget::updateDGainFromSpinBox(double value)
{
    int intValue = value*1000;
    // Temporarily disconnect signal-slot to avoid recursion between spinbox and slider
    disconnect( dGainSlider, SIGNAL(valueChanged(int)), this, SLOT(updateDGainFromSlider(int)) );
    dGainSlider->setValue(intValue);
    connect( dGainSlider, SIGNAL(valueChanged(int)), this, SLOT(updateDGainFromSlider(int)) );
    pid->setDGain(value);
}


void PidWidget::updateOutMaxFromSpinBox(double value)
{
    int intValue = value*1000;
    // Temporarily disconnect signal-slot to avoid recursion between spinbox and slider
    disconnect( outMaxSlider, SIGNAL(valueChanged(int)), this, SLOT(updateOutMaxFromSlider(int)) );
    outMaxSlider->setValue(intValue);
    connect( outMaxSlider, SIGNAL(valueChanged(int)), this, SLOT(updateOutMaxFromSlider(int)) );
    pid->setOutMax(value);
}


void PidWidget::updateOutMinFromSpinBox(double value)
{
    int intValue = value*1000;
    // Temporarily disconnect signal-slot to avoid recursion between spinbox and slider
    disconnect( outMinSlider, SIGNAL(valueChanged(int)), this, SLOT(updateOutMinFromSlider(int)) );
    outMinSlider->setValue(intValue);
    connect( outMinSlider, SIGNAL(valueChanged(int)), this, SLOT(updateOutMinFromSlider(int)) );
    pid->setOutMin(value);
}


void PidWidget::updatePGainFromSlider(int value)
{
    double doubleValue = value/1000.0;
    // Temporarily disconnect signal-slot to avoid recursion between spinbox and slider
    disconnect( pGainCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updatePGainFromSpinBox(double)) );
    pGainCurrentSpinBox->setValue(doubleValue);
    connect( pGainCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updatePGainFromSpinBox(double)) );
    pid->setPGain(doubleValue);
}


void PidWidget::updateIGainFromSlider(int value)
{
    double doubleValue = value/1000.0;
    // Temporarily disconnect signal-slot to avoid recursion between spinbox and slider
    disconnect( iGainCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateIGainFromSpinBox(double)) );
    iGainCurrentSpinBox->setValue(doubleValue);
    connect( iGainCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateIGainFromSpinBox(double)) );
    pid->setIGain(doubleValue);
}


void PidWidget::updateDGainFromSlider(int value)
{
    double doubleValue = value/1000.0;
    // Temporarily disconnect signal-slot to avoid recursion between spinbox and slider
    disconnect( dGainCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateDGainFromSpinBox(double)) );
    dGainCurrentSpinBox->setValue(doubleValue);
    connect( dGainCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateDGainFromSpinBox(double)) );
    pid->setDGain(doubleValue);
}


void PidWidget::updateOutMaxFromSlider(int value)
{
    double doubleValue = value/1000.0;
    // Temporarily disconnect signal-slot to avoid recursion between spinbox and slider
    disconnect( outMaxCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateOutMaxFromSpinBox(double)) );
    outMaxCurrentSpinBox->setValue(doubleValue);
    connect( outMaxCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateOutMaxFromSpinBox(double)) );
    pid->setOutMax(doubleValue);
}


void PidWidget::updateOutMinFromSlider(int value)
{
    double doubleValue = value/1000.0;
    // Temporarily disconnect signal-slot to avoid recursion between spinbox and slider
    disconnect( outMinCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateOutMinFromSpinBox(double)) );
    outMinCurrentSpinBox->setValue(doubleValue);
    connect( outMinCurrentSpinBox, SIGNAL(valueChanged(double)), this, SLOT(updateOutMinFromSpinBox(double)) );
    pid->setOutMin(doubleValue);
}
