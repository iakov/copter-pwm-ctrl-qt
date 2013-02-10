#include "MainWindow.hpp"
#include <QStyleFactory>
#include <QDebug>
#include <QStringList>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  a.setStyle(QStyleFactory::create("Cleanlooks"));

  MainWindow w;
  w.show();
  return a.exec();
}

