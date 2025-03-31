#include "GUI.h"
#include <QThread>
#include "MainThread.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QtWidgetsApplication1 mainWindow;

    ApplicationManager* constManager = new ApplicationManager();

    mainWindow.setWindowTitle("Interactive Map Editor");
    mainWindow.resize(1200, 800);
    mainWindow.show();
    //QtWidgetsApplication1::getInstance()->ptrManager = constManager;


    //constManager->handleStartTimer();
    //TimerControl control;
    
    //constManager->main = &mainWindow;
    //constManager->main->myGrid->stats.running;
    // 
    //manager.handleStartTimer();
    //manager.mainWindow = &mainWindow;

    return app.exec();
}


/*
// todo:
    1. Randomly Generate Nodes (don't worry about the obstacles)
    2. Generate Tours Based on incremental increases in nodes
    3. Generate Exporting Data (for now, just time, but later we want to track RAM and CPU utilization
        create a separate QThread and 



*/