#ifndef SENSORGRAPH_H
#define SENSORGRAPH_H

#include <qt5/QtCore/QString>
#include <qt5/QtCore/QStringList>
#include <qt5/QtCore/QVector>
#include <qt5/QtCore/QElapsedTimer>
#include <qt5/QtCore/QTimer>
#include <qt5/QtWidgets/QWidget>
#include <qt5/QtWidgets/QCheckBox>
#include "qcustomplot/qcustomplot.h"

class SensorGraph : public QWidget
{
    Q_OBJECT
public:
    explicit SensorGraph(QStringList lineNames = QStringList(), QString yLabel = "Value", QWidget* parent = 0);

signals:

public slots:
    void appendData(int index, double x, double y);
    void updateGraphs();
    void togglePause(bool checked);

private:
    void customiseLayout();
    QCustomPlot* customPlot;
    QCheckBox* pauseCheckBox;
    int numOfLines;
    QStringList lineNames;
    QVector< QVector<double> > bufferXData;
    QVector< QVector<double> > bufferYData;
    QElapsedTimer* elapsedTimer;
    QTimer* graphUpdateTimer;
};

#endif // SENSORGRAPH_H
