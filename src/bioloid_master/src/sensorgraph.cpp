#include "sensorgraph.h"
#include <qt5/QtGui/QPen>
#include <qt5/QtGui/QColor>
#include <qt5/QtWidgets/QHBoxLayout>


SensorGraph::SensorGraph(QStringList lineNames, QString yLabel, QWidget* parent) :
    numOfLines(lineNames.size()), lineNames(lineNames), bufferXData(numOfLines), bufferYData(numOfLines),
    QWidget(parent)
{
    customPlot = new QCustomPlot(this);

    for (int i = 0; i < numOfLines; ++i)
    {
        bufferXData[i].resize(1000);
        bufferYData[i].resize(1000);

        customPlot->addGraph();
    }

    if (numOfLines == 1)
    {
        //customPlot->graph(0)->setName("Data");
        customPlot->graph(0)->setPen(QPen("black"));
    }
    else
    {
        for (int i = 0; i < numOfLines; ++i)
        {
            customPlot->graph(i)->setName(lineNames[i]);
            if (i <= 18)  // Range of Qt::GlobalColor enum
                customPlot->graph(i)->setPen( QPen(Qt::GlobalColor(7 + i)) );
        }

        customPlot->legend->setVisible(true);

        QFont legendFont = font();
        legendFont.setPointSize(8);
        customPlot->legend->setFont(legendFont);
        customPlot->legend->setBrush( QBrush(QColor(255, 255, 255, 0)) );  // Transparent
        // By default, the legend is in the inset layout of the main axis rect.
        // So this is how we access it to change legend placement.
        customPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop | Qt::AlignRight);
    }

    customPlot->xAxis->setLabel("Time (msec)");
    customPlot->yAxis->setLabel(yLabel);

    pauseCheckBox = new QCheckBox("Paused");

    QHBoxLayout* hBoxLayout = new QHBoxLayout;
    hBoxLayout->addWidget(customPlot, 1);
    hBoxLayout->addWidget(pauseCheckBox, 0);
    setLayout(hBoxLayout);

    customiseLayout();

    customPlot->replot();

    elapsedTimer = new QElapsedTimer();
    graphUpdateTimer = new QTimer(this);

    connect( graphUpdateTimer, SIGNAL(timeout()), this, SLOT(updateGraphs()) );
    connect( pauseCheckBox, SIGNAL(toggled(bool)), this, SLOT(togglePause(bool)) );

    elapsedTimer->start();
    graphUpdateTimer->start(10);
}


void SensorGraph::appendData(int index, double x, double y)
{
    bufferXData[index].pop_front();
    bufferYData[index].pop_front();
    bufferXData[index].push_back(x);
    bufferYData[index].push_back(y);
}


void SensorGraph::updateGraphs()
{
    double tInMsec = elapsedTimer->elapsed();
    for (int i = 0; i < numOfLines; ++i)
    {
        customPlot->graph(i)->setData(bufferXData[i], bufferYData[i]);
    }
    customPlot->rescaleAxes();
    customPlot->xAxis->setRange(tInMsec - 10000, tInMsec);
    customPlot->replot();
}


void SensorGraph::togglePause(bool checked)
{
    if (checked)
    {
        // Pause
        graphUpdateTimer->stop();
        // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking
        customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    }
    else
    {
        // Unpause
        graphUpdateTimer->start();
        // Disable user interaction
        customPlot->setInteractions(0);
    }
}


void SensorGraph::customiseLayout()
{
    customPlot->setBackground(QColor("lightgrey"));
}
