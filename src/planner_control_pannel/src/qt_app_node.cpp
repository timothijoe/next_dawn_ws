// #include <draw_qt_picture/qt_app_node.h>
#include "control_pannel.h"
#include <QApplication>

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    ControlPannel w;
    w.show();
    return a.exec();
}
