#ifndef MOTORVALUEEDITOR_H
#define MOTORVALUEEDITOR_H

#include <qt5/QtCore/QVector>
#include <qt5/QtCore/QMap>
#include <qt5/QtCore/QString>
#include <qt5/QtWidgets/QWidget>
#include <qt5/QtWidgets/QComboBox>
#include <qt5/QtWidgets/QLineEdit>
#include <qt5/QtWidgets/QDoubleSpinBox>
#include <qt5/QtWidgets/QPushButton>
#include "rosworker.h"

class MotorValueEditor : public QWidget
{
    Q_OBJECT

public:
    explicit MotorValueEditor(RosWorker* rosWorker, QWidget* parent = 0);

signals:

public slots:
    void updateOption(const QString &text);
    void getValue(int dxlId);
    void setValue(int dxlId);

private:
    void populateMap(QMap<QString, int>* inputMap);
    QMap<QString, int> optionsMap;
    QComboBox* optionsComboBox;
    QVector<QLineEdit*> currentValueLineEdits;
    QVector<QDoubleSpinBox*> goalValueSpinBoxes;
    QVector<QPushButton*> getValueButtons;
    QVector<QPushButton*> setValueButtons;
    RosWorker* mRosWorker;
    int mSelectedControlTableAddress;
};

#endif // MOTORVALUEEDITOR_H
