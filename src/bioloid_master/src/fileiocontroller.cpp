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

    customiseLayout();
}


void FileIoController::customiseLayout()
{
    QString buttonStyleSheet =
            "QPushButton {"
            "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
            "stop: 0 lightsteelblue, stop: 1 steelblue);"
            "border-color: #8F8F91;"
            "border-style: outset;"
            "border-width: 4px;"
            "border-radius: 10px; }"
            "QPushButton:flat {"
            "border: none;"  /* no border for a flat push button */
            "}"
            "QPushButton:default {"
            "border-color: navy;"  /* make the default button prominent */
            "}"
            "QPushButton:pressed {"
            "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
            "stop: 0 royalblue, stop: 1 dodgerblue);"
            "border-style: inset; }";

    saveAvailablePosesFileButton->setStyleSheet(buttonStyleSheet);
    saveQueuedPosesFileButton->setStyleSheet(buttonStyleSheet);
    loadAvailablePosesFileButton->setStyleSheet(buttonStyleSheet);
    loadQueuedPosesFileButton->setStyleSheet(buttonStyleSheet);
}
