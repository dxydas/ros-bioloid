#include "customlistwidget.h"
#include <qt5/QtCore/QDir>
#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QAbstractItemView>
#include <qt5/QtWidgets/QFileDialog>
#include <qt5/QtWidgets/QMessageBox>
#include <qt5/QtWidgets/QAbstractButton>


CustomListWidget::CustomListWidget(QList<RobotPose> posesList, QString title,
                                   bool allowDuplNames, QWidget *parent) :
    QWidget(parent)
{
    QGridLayout* layout = new QGridLayout;
    setLayout(layout);

    QLabel* label = new QLabel(title);

    mRobotPosesListModel = new RobotPosesListModel(posesList, allowDuplNames);

    mListView = new QListView(this);
    mListView->setModel(mRobotPosesListModel);
    mListView->setSelectionMode(QAbstractItemView::SingleSelection);

    moveUpButton = new QPushButton("Move up");
    moveDownButton = new QPushButton("Move down");

    int row = 0;
    layout->addWidget(label, row++, 0, 1, -1);
    layout->addWidget(mListView, row++, 0, 1, -1);
    layout->addWidget(moveUpButton, row, 0);
    layout->addWidget(moveDownButton, row++, 1);

    customiseLayout();

    connect( moveUpButton, SIGNAL(clicked()), this, SLOT(moveUp()) );
    connect( moveDownButton, SIGNAL(clicked()), this, SLOT(moveDown()) );
}


void CustomListWidget::customiseLayout()
{
    QString listViewStyleSheet =
            ( "QListView {"
              "background-color: grey;"
              "show-decoration-selected: 1; }"  // make the selection span the entire width of the view
              "QListView::item:alternate {"
              "background: lightslategrey; }"
              "QListView::item:selected {"
              "border: 1px solid rgb(50, 110, 160); }"
              "QListView::item:selected:!active {"
              "background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
              "stop: 0 lightsteelblue, stop: 1 steelblue); }"
              "QListView::item:selected:active {"
              "background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
              //"stop: 0 royalblue, stop: 1 dodgerblue); }"
              "stop: 0 darkorange, stop: 1 orange); }"
              "QListView::item:hover {"
              "background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
              "stop: 0 rgb(218, 230, 240), stop: 1 rgb(144, 180, 210)); }" );

    QString buttonStyleSheet =
            ( "QPushButton {"
              "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
              "stop: 0 lightsteelblue, stop: 1 steelblue);"
              "border: solid #8F8F91;"
              "border-style: outset;"
              "border-width: 4px;"
              "border-radius: 10px; }"
              //"border-color: beige; }"
              //"font: bold 14px; }"
              //"min-width: 10em;"
              //"padding: 6px; }"
              "QPushButton:flat {"
              "border: none;"  /* no border for a flat push button */
              "}"
              "QPushButton:default {"
              "border-color: navy;"  /* make the default button prominent */
              "}"
              "QPushButton:pressed {"
              "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
              "stop: 0 royalblue, stop: 1 dodgerblue);"
              "border-style: inset; }" );

    mListView->setAlternatingRowColors(true);
    moveUpButton->setIcon( QIcon("assets/images/ionicons-2.0.1/src/arrow-up-b.svg") );
    moveDownButton->setIcon( QIcon("assets/images/ionicons-2.0.1/src/arrow-down-b.svg") );

    mListView->setStyleSheet(listViewStyleSheet);
    moveUpButton->setStyleSheet(buttonStyleSheet);
    moveDownButton->setStyleSheet(buttonStyleSheet);
}


void CustomListWidget::add(RobotPose robotPose)
{
    QModelIndex index = mListView->currentIndex();
    int err = mRobotPosesListModel->add(robotPose, index);
    if (err == 1)
    {
        QMessageBox::warning(this, "Failed to add pose",
                             "Cannot add pose with empty name.");
        return;
    }
    else if (err == 2)
    {
        QMessageBox::warning(this, "Failed to add pose",
                             "Pose name must be unique.");
        return;
    }
    mListView->setCurrentIndex(index);
}


void CustomListWidget::addFrom(CustomListWidget* otherCustomListWidget)
{
    QModelIndex index = mListView->currentIndex();
    QModelIndex otherIndex = otherCustomListWidget->getListView()->currentIndex();
    mRobotPosesListModel->add(otherCustomListWidget->getRobotPosesListModel()->getCurrentPose(otherIndex), index);
    mListView->setCurrentIndex(index);
}


void CustomListWidget::remove()
{
    QModelIndex index = mListView->currentIndex();
    mRobotPosesListModel->remove(index);
    mListView->setCurrentIndex(index);
}


void CustomListWidget::moveUp()
{
    QModelIndex index = mListView->currentIndex();
    mRobotPosesListModel->moveUp(index);
    mListView->setCurrentIndex(index);
}


void CustomListWidget::moveDown()
{
    QModelIndex index = mListView->currentIndex();
    mRobotPosesListModel->moveDown(index);
    mListView->setCurrentIndex(index);
}


void CustomListWidget::savePosesFile()
{
    //QString fileName = QFileDialog::getSaveFileName(this, "Save poses", QDir::homePath(), "Pose files (*.txt)");
    QFileDialog dialog(this, "Save poses", QDir::homePath());
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setNameFilter("Pose files (*.txt)");
    dialog.setViewMode(QFileDialog::Detail);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setDefaultSuffix("txt");
    QString fileName;
    if (dialog.exec())
        fileName = dialog.selectedFiles().first();

    mRobotPosesListModel->savePosesFile(fileName);
}


void CustomListWidget::loadPosesFile()
{
    //QString fileName = QFileDialog::getOpenFileName(this, "Load poses", QDir::homePath(), "Pose files (*.txt)");
    QFileDialog dialog(this, "Load poses", QDir::homePath());
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setNameFilter("Pose files (*.txt)");
    dialog.setViewMode(QFileDialog::Detail);
    dialog.setAcceptMode(QFileDialog::AcceptOpen);
    QString fileName;
    if (dialog.exec())
        fileName = dialog.selectedFiles().first();

    QModelIndex index;
    mRobotPosesListModel->loadPosesFile(fileName, index);
    mListView->setCurrentIndex(index);
}
