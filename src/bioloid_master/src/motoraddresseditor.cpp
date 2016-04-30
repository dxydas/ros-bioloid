#include "motoraddresseditor.h"
#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QVariant>
#include <qt5/QtCore/QTimer>
#include <qt5/QtWidgets/QGridLayout>
#include <qt5/QtWidgets/QFormLayout>
#include <qt5/QtWidgets/QHBoxLayout>
#include <qt5/QtWidgets/QSizePolicy>
#include <qt5/QtWidgets/QMessageBox>
#include <qt5/QtWidgets/QSpinBox>
#include <qt5/QtWidgets/QApplication>
#include "commonvars.h"
#include "../../usb2ax_controller/src/ax12ControlTableMacros.h"


SpinBoxDelegate::SpinBoxDelegate(QObject* parent) :
    QStyledItemDelegate(parent)
{
}


QWidget* SpinBoxDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem &option,
                                       const QModelIndex &index) const
{
    QSpinBox* editor = new QSpinBox(parent);
    editor->setFrame(false);
    editor->setMinimum(0);
    editor->setMaximum(2047);

    return editor;
}


void SpinBoxDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
    int value = index.model()->data(index, Qt::DisplayRole).toInt();

    QSpinBox* spinBox = static_cast<QSpinBox*>(editor);
    spinBox->setValue(value);
}


void SpinBoxDelegate::setModelData(QWidget *editor, QAbstractItemModel *model,
                                   const QModelIndex &index) const
{
    QSpinBox* spinBox = static_cast<QSpinBox*>(editor);
    spinBox->interpretText();
    int value = spinBox->value();

    model->setData(index, value, Qt::UserRole);
}


void SpinBoxDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option,
                                           const QModelIndex &index) const
{
    editor->setGeometry(option.rect);
}


MotorAddressEditor::MotorAddressEditor(RosWorker* rosWorker, QWidget* parent) :
    rosWorker(rosWorker), QWidget(parent), selectedMotor(1)
{
    setWindowTitle("Motor Address Editor");



    motorComboBox = new QComboBox;
    for (int dxlId = 1; dxlId <= NUM_OF_MOTORS; ++dxlId)
        motorComboBox->addItem(QString::number(dxlId));
    motorComboBox->setCurrentIndex(0);
    motorComboBox->setMinimumWidth(60);

    QFormLayout* optionsFormLayout = new QFormLayout;
    optionsFormLayout->addRow("Select motor:", motorComboBox);

    refreshButton = new QPushButton("Refresh");

    refreshLabel = new QLabel("Refreshing ...");
    refreshLabel->setVisible(false);



    controlTableModel = new ControlTableModel(this);

    tableView = new QTableView;
    tableView->setModel(controlTableModel);
    tableView->resizeColumnsToContents();

    // Spin box delegate for editable values (column 4)
    SpinBoxDelegate* spinBoxDelegate = new SpinBoxDelegate;
    tableView->setItemDelegateForColumn(4, spinBoxDelegate);






    int row = 0;
    int col = 0;
    QGridLayout* gridLayout = new QGridLayout;
    gridLayout->addLayout(optionsFormLayout, row++, col, 1, 1, Qt::AlignTop);
    gridLayout->addWidget(refreshButton, row++, col, 1, 1, Qt::AlignTop);
    gridLayout->addWidget(refreshLabel, row++, col, 1, 1, Qt::AlignTop);
    gridLayout->setAlignment(Qt::AlignTop);


    QHBoxLayout* hBoxLayout = new QHBoxLayout;
    hBoxLayout->addWidget(tableView);
    hBoxLayout->addLayout(gridLayout);





    setMinimumSize(800, 600);

    setLayout(hBoxLayout);

    customiseLayout();

    connect( motorComboBox, SIGNAL(currentIndexChanged(int)),
             this, SLOT(updateSelectedMotor(int)) );
    connect( refreshButton, SIGNAL(clicked()), this, SLOT(refreshData()) );
    connect( controlTableModel, SIGNAL(sendData(QString, int, int)),
             this, SLOT(setValuePrompt(QString, int, int)) );
}


void MotorAddressEditor::updateSelectedMotor(int index)
{
    selectedMotor = index + 1;
    refreshData();
}


void MotorAddressEditor::refreshData()
{
    // Flash refresh label for 0.5 a sec
    refreshLabel->setVisible(true);
    QTimer::singleShot( 500, this, SLOT(hideRefreshLabel()) );

    usb2ax_controller::ReceiveFromAX srv;
    for (int r = 0; r < controlTableModel->rowCount(); ++r)
    {
        srv.request.dxlID = selectedMotor;
        srv.request.address = controlTableModel->getAddress(r);
        rosWorker->receiveFromAXClient.call(srv);
        controlTableModel->setData(controlTableModel->index(r, 4), srv.response.value, Qt::EditRole);
    }
}


void MotorAddressEditor::setValuePrompt(QString name, int address, int value)
{
    if (selectedMotor < 0)
    {
        refreshData();
        return;
    }


    //int address = controlTableModel->getAddress(topLeft.row());
    //int targetValue = controlTableModel->getTargetValue(topLeft.row());

    //int address = controlTableModel->index(index.row(), 1).data().toUInt();
    //int targetValue = index.data().toUInt();


    QMessageBox::StandardButton reply = QMessageBox::warning(
                this, "Set value?",
                "Set " + name + " of motor with ID " + QString::number(selectedMotor) +
                " to " + QString::number(value) + "?",
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (reply == QMessageBox::No)
    {
        refreshData();
        return;
    }


    if ( (address == AX12_ID) || (address == AX12_BAUD_RATE) )
    {
        QMessageBox::StandardButton reply = QMessageBox::warning(
                    this, "Confirm?",
                    "Are you really sure?\nThis is usually a bad idea!",
                    QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (reply == QMessageBox::No)
        {
            refreshData();
            return;
        }
    }

    usb2ax_controller::SendToAX srv;
    srv.request.dxlID = selectedMotor;
    srv.request.address = address;
    srv.request.value = value;
    rosWorker->sendtoAXClient.call(srv);
    refreshData();
}


void MotorAddressEditor::customiseLayout()
{
    QString tableViewStyleSheet =
            "QTableView {"
            "border: 2px solid steelblue;"
            "color: black;"
            "background-color: lightgrey; }"
            // item:selected must be declared before item:focus
            "QTableView:item:selected {"
            "background: steelblue; }"
            "QTableView:item:focus {"
            "background: dodgerblue; }";

    QString comboBoxStyleSheet =
            "QComboBox {"
            "border: 2px solid steelblue;"
            "color: black;"
            "background-color: lightslategrey;"
            "selection-color: white;"
            "selection-background-color: lightslategrey; }" ;

    QString buttonStyleSheet =
            "QPushButton {"
            "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
            "stop: 0 lightsteelblue, stop: 1 steelblue);"
            "border-color: #8F8F91;"
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
            "border-style: inset; }" ;

    QString labelStyleSheet =
            "QLabel {"
            "color: blue; }" ;

    tableView->setStyleSheet(tableViewStyleSheet);
    motorComboBox->setStyleSheet(comboBoxStyleSheet);
    refreshButton->setStyleSheet(buttonStyleSheet);
    refreshLabel->setStyleSheet(labelStyleSheet);
}
