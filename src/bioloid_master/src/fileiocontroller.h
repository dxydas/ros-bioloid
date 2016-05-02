#ifndef FILEIOCONTROLLER_H
#define FILEIOCONTROLLER_H

#include <qt5/QtWidgets/QWidget>
#include <qt5/QtWidgets/QPushButton>

class FileIoController : public QWidget
{
    Q_OBJECT

public:
    explicit FileIoController(QWidget* parent = 0);
    QPushButton* saveAvailablePosesFileButton;
    QPushButton* saveQueuedPosesFileButton;
    QPushButton* loadAvailablePosesFileButton;
    QPushButton* loadQueuedPosesFileButton;

signals:

public slots:

private:
};

#endif // FILEIOCONTROLLER_H
