#include "fileiocontroller.h"
#include <qt5/QtWidgets/QGridLayout>


FileIoController::FileIoController(QWidget* parent) :
    QWidget(parent)
{
    saveAvailablePosesFileButton = new QPushButton("Save available poses file");
    saveQueuedPosesFileButton = new QPushButton("Save queued poses file");
    loadAvailablePosesFileButton = new QPushButton("Load available poses file");
    loadQueuedPosesFileButton = new QPushButton("Load queued poses file");

    QGridLayout* fileIoSubLayout = new QGridLayout;
    int row = 0;
    int col = 0;
    fileIoSubLayout->addWidget(saveAvailablePosesFileButton, row, col++);
    fileIoSubLayout->addWidget(saveQueuedPosesFileButton, row++, col--);
    fileIoSubLayout->addWidget(loadAvailablePosesFileButton, row, col++);
    fileIoSubLayout->addWidget(loadQueuedPosesFileButton, row, col--);
    fileIoSubLayout->setAlignment(Qt::AlignTop);

//    QWidget* fileIoWidget =  new QWidget(this);
//    fileIoWidget->setLayout(fileIoSubLayout);
    setLayout(fileIoSubLayout);
}
