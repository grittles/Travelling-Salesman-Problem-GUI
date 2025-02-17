#include "GUI.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QtWidgetsApplication1 mainWindow;
    mainWindow.setWindowTitle("Interactive Map Editor");
    mainWindow.resize(1200, 800);
    mainWindow.show();
    return app.exec();
}
