#include "pclviewer.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  PCLViewer w;
  // http://doc.qt.io/qt-5/qtwidgets-widgets-windowflags-example.html
  // http://doc.qt.io/qt-5/qt.html#WindowType-enum
  // https://stackoverflow.com/questions/28319600/enable-maximize-button-in-qwizard    <--- this one! need for CustomizeWindowHint
  w.setWindowFlags(Qt::Window | Qt::CustomizeWindowHint | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint);
  // https://stackoverflow.com/questions/19817881/qt-fullscreen-on-startup
  w.setWindowState(Qt::WindowMaximized);
  // w.setWindowState(Qt::WindowFullScreen);
  w.show();
  return a.exec ();
}
