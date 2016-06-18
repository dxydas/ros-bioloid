#ifndef MOTORADDRESSEDITOR_H
#define MOTORADDRESSEDITOR_H

#include <qt5/QtCore/QtCore>
#include <qt5/QtCore/QVector>
#include <qt5/QtCore/QModelIndex>
#include <qt5/QtWidgets/QWidget>
#include <qt5/QtWidgets/QStyledItemDelegate>
#include <qt5/QtWidgets/QTableView>
#include <qt5/QtWidgets/QComboBox>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QLabel>
#include "controltablemodel.h"
#include "rosworker.h"

class SpinBoxDelegate : public QStyledItemDelegate
{
    Q_OBJECT

public:
    SpinBoxDelegate(QObject *parent = 0);
    QWidget* createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                          const QModelIndex &index) const Q_DECL_OVERRIDE;
    void setEditorData(QWidget *editor, const QModelIndex &index) const Q_DECL_OVERRIDE;
    void setModelData(QWidget *editor, QAbstractItemModel *model,
                      const QModelIndex &index) const Q_DECL_OVERRIDE;
    void updateEditorGeometry(QWidget *editor,const QStyleOptionViewItem &option,
                              const QModelIndex &index) const Q_DECL_OVERRIDE;
};

class MotorAddressEditor : public QWidget
{
    Q_OBJECT

public:
    explicit MotorAddressEditor(RosWorker* rosWorker, QWidget *parent = 0);

signals:

public slots:
    void updateSelectedMotor(int index);
    void refreshData();
    void hideRefreshLabel() { refreshLabel->setVisible(false); }
    void setValuePrompt(QString name, int address, int value);

private:
    QPushButton* refreshButton;
    QLabel* refreshLabel;
    QTableView* tableView;
    QComboBox* motorComboBox;
    RosWorker* rosWorker;
    int selectedMotor;
    ControlTableModel* controlTableModel;
};

#endif // MOTORADDRESSEDITOR_H
