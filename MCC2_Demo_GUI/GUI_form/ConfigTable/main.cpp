#include "configtable.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ConfigTable w;
    w.show();
    return a.exec();
}
