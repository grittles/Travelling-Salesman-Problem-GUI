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


/*
// todo:
    1. Randomly Generate Nodes (don't worry about the obstacles)
    2. Generate Tours Based on incremental increases in nodes
    3. Generate Exporting Data (for now, just time, but later we want to track RAM and CPU utilization
        create a separate QThread and 



*/