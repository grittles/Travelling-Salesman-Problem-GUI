#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QtWidgetsApplication1.h"

#include <QtWidgets/QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QPushButton>
#include <QTextEdit>
#include <QSplitter>

#include "Grid.h"
#include "MapScene.h"
#include "CustomView.h"
#include <QLineEdit>

#include <chrono>  // Include the chrono library

class QtWidgetsApplication1 : public QMainWindow
{
    Q_OBJECT

public:
    QtWidgetsApplication1(QWidget *parent = nullptr);
    ~QtWidgetsApplication1();

    // Static function to access the instance
    static QtWidgetsApplication1* getInstance();
    Grid* getGrid();

    int sceneWidth;
    int sceneHeight;
    int bSize = 10;
    Grid* myGrid = nullptr;
    QWidget* _parent = nullptr;


private:
    Ui::QtWidgetsApplication1Class ui;
    //QGraphicsScene* mapScene;
    //QGraphicsView* mapView;

    MapScene* mapScene; // Use CustomScene
    CustomView* mapView;    // Custom QGraphicsView for zoom/pan

    QSplitter* mainSplitter;
    QSplitter* topSplitter;

    QWidget* centralWidget;
    QVBoxLayout* mainLayout;
    QHBoxLayout* mainSplitLayout;

    QComboBox* dropdown;
    QPushButton* saveButton;
    QPushButton* openButton;
    QPushButton* resizeButton;
    QPushButton* resetButton;
    QPushButton* syncGridButton;
    QPushButton* syncCanvasButton;
    QPushButton* printGridButton;
    QPushButton* tracePathButton;
    QPushButton* traceAllPathsButton;
    QPushButton* traceGreedyPathButton;

    QLineEdit* minSizeEdit;
    QLineEdit* maxSizeEdit;

    QTextEdit* console;

    QWidget* rightPanel;
    QVBoxLayout* rightLayout;

    // Static instance pointer
    static QtWidgetsApplication1* instance;
    int selectedDropType = 0;

signals:
    void requestCanvasInitialization(int width, int height, int blockSize);

public slots:
    void WriteConsole(QString string);
    int getType();
    void ResizeCanvas(int gridToCanvas);
    void ResizeCanvasArgs(int minsize, int maxsize, int gridToCanvas);

private slots:
    void handleDropdownChange(int index);

    void saveToFile();
    void openFromFile();
    void InitCanvas();
    void syncGrid();
    void syncCanvas();
    void printGrid();
    void tracePath();
    void traceAllPaths();
    void traceGreedyPath();
    
};
