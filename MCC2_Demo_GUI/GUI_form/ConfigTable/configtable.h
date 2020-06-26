#ifndef CONFIGTABLE_H
#define CONFIGTABLE_H

#include <QWidget>

QT_BEGIN_NAMESPACE
namespace Ui { class ConfigTable; }
QT_END_NAMESPACE

class ConfigTable : public QWidget
{
    Q_OBJECT

public:
    ConfigTable(QWidget *parent = nullptr);
    ~ConfigTable();

private:
    Ui::ConfigTable *ui;
};
#endif // CONFIGTABLE_H
