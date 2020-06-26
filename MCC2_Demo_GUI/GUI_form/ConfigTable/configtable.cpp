#include "configtable.h"
#include "ui_configtable.h"

ConfigTable::ConfigTable(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ConfigTable)
{
    ui->setupUi(this);
}

ConfigTable::~ConfigTable()
{
    delete ui;
}

