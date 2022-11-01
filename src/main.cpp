#include "CloudViewer.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  CloudViewer w;
  w.show();
  return a.exec();

  // cmake -H. -Bbuild -DCMAKE_EXPORT_COMPILE_COMMANDS=on
}

