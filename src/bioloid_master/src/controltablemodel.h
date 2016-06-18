#ifndef CONTROLTABLEMODEL_H
#define CONTROLTABLEMODEL_H

#include <qt5/QtCore/Qt>
#include <qt5/QtCore/QtCore>
#include <qt5/QtCore/QtGlobal>
#include <qt5/QtCore/QVariant>
#include <qt5/QtCore/QModelIndex>
#include <qt5/QtCore/QAbstractTableModel>
#include <qt5/QtCore/QVector>

struct ControlTableRow
{
    QString area;
    int address;
    QString name;
    QString description;
    bool writeAccess;
    int value;
    //int targetValue;
};

class ControlTableModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    explicit ControlTableModel(QObject *parent = 0);
    int rowCount(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;
    int columnCount(const QModelIndex &parent = QModelIndex()) const Q_DECL_OVERRIDE;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const Q_DECL_OVERRIDE;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) Q_DECL_OVERRIDE;
    Qt::ItemFlags flags(const QModelIndex &index) const Q_DECL_OVERRIDE;
    int getAddress(int row) { return controlTableRows[row]->address; }
    QString getName(int row) { return controlTableRows[row]->name; }
//    int getTargetValue(int row) { return controlTableRows[row]->targetValue; }

signals:
    void sendData(QString name, int address, int value);

public slots:

private:
    void initControlTableRows();
    QVector<ControlTableRow*> controlTableRows;
};

#endif // CONTROLTABLEMODEL_H
