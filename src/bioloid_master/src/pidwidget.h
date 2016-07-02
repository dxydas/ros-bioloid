#ifndef PIDWIDGET_H
#define PIDWIDGET_H

#include <qt5/QtWidgets/QWidget>
#include <qt5/QtWidgets/QGroupBox>
#include <qt5/QtWidgets/QDoubleSpinBox>
#include <qt5/QtWidgets/QAbstractSlider>
#include <qt5/QtWidgets/QSlider>
#include "simplepid.h"

class PidWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PidWidget(SimplePid* pid, QWidget* parent = 0);

signals:

public slots:
    void pGainMaxUpdated(double value);
    void iGainMaxUpdated(double value);
    void dGainMaxUpdated(double value);
    void outMaxMaxUpdated(double value);
    void outMinMaxUpdated(double value);
    void pGainMinUpdated(double value);
    void iGainMinUpdated(double value);
    void dGainMinUpdated(double value);
    void outMaxMinUpdated(double value);
    void outMinMinUpdated(double value);
    void updatePGainFromSpinBox(double value);
    void updateIGainFromSpinBox(double value);
    void updateDGainFromSpinBox(double value);
    void updateOutMaxFromSpinBox(double value);
    void updateOutMinFromSpinBox(double value);
    void updatePGainFromSlider(int value);
    void updateIGainFromSlider(int value);
    void updateDGainFromSlider(int value);
    void updateOutMaxFromSlider(int value);
    void updateOutMinFromSlider(int value);

private:
    QGroupBox* pGainGroupBox;
    QGroupBox* iGainGroupBox;
    QGroupBox* dGainGroupBox;
    QGroupBox* outMaxGroupBox;
    QGroupBox* outMinGroupBox;
    SimplePid* pid;
    QDoubleSpinBox* pGainMaxSpinBox;
    QDoubleSpinBox* iGainMaxSpinBox;
    QDoubleSpinBox* dGainMaxSpinBox;
    QDoubleSpinBox* outMaxMaxSpinBox;
    QDoubleSpinBox* outMinMaxSpinBox;
    QDoubleSpinBox* pGainMinSpinBox;
    QDoubleSpinBox* iGainMinSpinBox;
    QDoubleSpinBox* dGainMinSpinBox;
    QDoubleSpinBox* outMaxMinSpinBox;
    QDoubleSpinBox* outMinMinSpinBox;
    QDoubleSpinBox* pGainCurrentSpinBox;
    QDoubleSpinBox* iGainCurrentSpinBox;
    QDoubleSpinBox* dGainCurrentSpinBox;
    QDoubleSpinBox* outMaxCurrentSpinBox;
    QDoubleSpinBox* outMinCurrentSpinBox;
    QSlider* pGainSlider;
    QSlider* iGainSlider;
    QSlider* dGainSlider;
    QSlider* outMaxSlider;
    QSlider* outMinSlider;
};

#endif // PIDWIDGET_H
