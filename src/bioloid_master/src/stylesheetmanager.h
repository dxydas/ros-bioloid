#ifndef STYLESHEETMANAGER_H
#define STYLESHEETMANAGER_H

#include <qt5/QtWidgets/QObject>

class StyleSheetManager : public QObject
{
    Q_OBJECT

public:
    explicit StyleSheetManager(QObject* parent = 0);

signals:

public slots:

private:
    void loadStyleSheets();
};

#endif // STYLESHEETMANAGER_H
