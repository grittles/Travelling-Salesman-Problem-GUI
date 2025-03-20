#include "GUI.h"



// Define the static instance (This is for linking to work)
QtWidgetsApplication1* QtWidgetsApplication1::instance = nullptr;

void QtWidgetsApplication1::handleDropdownChange(int index)
{
    
    selectedDropType = index;

    QString selectedType = QString("Selected Type %1").arg(index + 1);
    WriteConsole(selectedType);
}

int QtWidgetsApplication1::getType()
{
    return selectedDropType;
}

void QtWidgetsApplication1::saveToFile()
{

    QString word = QString("Attempting to Save File");
    WriteConsole(word);

    myGrid->saveFile(_parent);
}

void QtWidgetsApplication1::openFromFile()
{
    QString word = QString("Opening File");
    WriteConsole(word);

    myGrid->openFile(_parent);

    syncGrid();
}

void QtWidgetsApplication1::ResizeCanvas(int gridToCanvas)
{
    bool okMin, okMax;

    int minSize = minSizeEdit->text().toInt(&okMin);
    int maxSize = maxSizeEdit->text().toInt(&okMax);

    // this could cause some real problems if this bool check fails, beware.
    if (okMin && okMax) {
        ResizeCanvasArgs(minSize, maxSize, gridToCanvas);
    }
    else {
        WriteConsole("Invalid input for resizing");
    }
}

void QtWidgetsApplication1::ResizeCanvasArgs(int minSize = -1, int maxSize = -1, int gridToCanvas = false)
{

    // minSize = std::round(static_cast<double>(minSize) / 10)* 10; // round to nearest 10.
    // maxSize = std::round(static_cast<double>(maxSize) / 10) * 10; // round to nearest 10.

    // Assuming you want to resize the scene rect
    // we're just going to assume blocksize is 10 at this point because what else would it be?
    mapScene->setSceneRect(0, 0, minSize*10, maxSize*10);

    sceneWidth = minSize*10;
    sceneHeight = maxSize*10;

    mapView->resizeCanvas(gridToCanvas);

    QString resizeMsg = QString("Canvas resized to min: %1, max: %2").arg(minSize).arg(maxSize);
    WriteConsole(resizeMsg);
}

void QtWidgetsApplication1::InitCanvas()
{
    mapView->resetCanvas(sceneWidth, sceneHeight, bSize);
}

void QtWidgetsApplication1::syncGrid()
{
    mapView->syncGridToCanvas();
}

void QtWidgetsApplication1::syncCanvas()
{
    mapView->syncCanvasToGrid();
}

void QtWidgetsApplication1::printGrid()
{
    QString text = myGrid->printGrid();
    WriteConsole(text);
}

void QtWidgetsApplication1::tracePath()
{
    syncCanvas();
    // if I were to redo this in C rather than C++ I'd probably trace my path as a linked list
    // I planned on doing the GUI in C++ and doing the pathing and algorithms in C. Kinda off
    // my original plan at this point.

    //QString text = myGrid->PrintPath(path);
    //WriteConsole(text);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = myGrid->TSPSolve_heldKarp();
    int distance = result.first;

    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    QString text;

    text = QString("Time taken to Execute: %1 ms").arg(duration);
    WriteConsole(text);

    auto c = myGrid->origin;

    if (c != nullptr) {
        text = QString("origin: [%1, %2]").arg(c->x).arg(c->y);
        WriteConsole(text);
    }

    text = myGrid->PrintPath(result.second);
    WriteConsole(text);

    text = QString("Total Distance: %1").arg(distance);
    WriteConsole(text);

    syncGrid();
}

void QtWidgetsApplication1::traceAllPaths()
{
    syncCanvas();
    // if I were to redo this in C rather than C++ I'd probably trace my path as a linked list
    // I planned on doing the GUI in C++ and doing the pathing and algorithms in C. Kinda off
    // my original plan at this point.
    
    //QString text = myGrid->PrintPath(path);
    //WriteConsole(text);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = myGrid->TSPSolve_LK();
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    QString text;

    text = QString("Time taken to Execute: %1 ms").arg(duration);
    WriteConsole(text);

    auto c = myGrid->origin;

    if (c != nullptr) {
        text = QString("origin: [%1, %2]").arg(c->x).arg(c->y);
        WriteConsole(text);
    }

    text = myGrid->PrintPath(result.second);
    WriteConsole(text);

    text = QString("Total Distance: %1").arg(result.first);
    WriteConsole(text);

    syncGrid();
}


void QtWidgetsApplication1::traceGreedyPath()
{
    syncCanvas();
    // if I were to redo this in C rather than C++ I'd probably trace my path as a linked list
    // I planned on doing the GUI in C++ and doing the pathing and algorithms in C. Kinda off
    // my original plan at this point.

    //QString text = myGrid->PrintPath(path);
    //WriteConsole(text);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = myGrid->TSPSolve_Greedy();
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    QString text;

    text = QString("Time taken to Execute: %1 ms").arg(duration);
    WriteConsole(text);

    auto c = myGrid->origin;

    if (c != nullptr) {
        text = QString("origin: [%1, %2]").arg(c->x).arg(c->y);
        WriteConsole(text);
    }

    text = myGrid->PrintPath(result.second);
    WriteConsole(text);

    text = QString("Total Distance: %1").arg(result.first);
    WriteConsole(text);

    syncGrid();
}

void QtWidgetsApplication1::WriteConsole(QString string)
{
    console->append(string);
}

QtWidgetsApplication1* QtWidgetsApplication1::getInstance()
{
    return instance;
}

Grid* QtWidgetsApplication1::getGrid()
{
    return myGrid;
}

QtWidgetsApplication1::QtWidgetsApplication1(QWidget* parent)
    : QMainWindow(parent)
{
    Ui::QtWidgetsApplication1Class ui{}; // stupid warning

    ui.setupUi(this);

    instance = this;
    _parent = parent;

    // Central widget and main layout
    centralWidget = new QWidget(this);
    mainLayout = new QVBoxLayout(centralWidget);

    // Use a QSplitter for flexible resizing
    mainSplitter = new QSplitter(Qt::Vertical, this); // Vertical splitter

    // Map and Right Panel in another splitter for horizontal adjustments
    topSplitter = new QSplitter(Qt::Horizontal, this);
    mainSplitter->addWidget(topSplitter);

    // 1. Map Area
    mapScene = new MapScene(this);
    mapView = new CustomView(this);
    //mapView->setGUIParent(&this);
    mapView->setScene(mapScene);
    mapView->setRenderHint(QPainter::Antialiasing);
    mapScene->setSceneRect(0, 0, 800, 600); // default is 800x600 or 80x60

    QRectF sceneRect = mapScene->sceneRect(); // Get the scene rectangle
    sceneWidth = static_cast<int>(std::round(sceneRect.width()));
    sceneHeight = static_cast<int>(std::round(sceneRect.height()));

    topSplitter->addWidget(mapView);

    // 2. Right Panel
    rightPanel = new QWidget(this);
    rightLayout = new QVBoxLayout(rightPanel);

    dropdown = new QComboBox(this);
    dropdown->addItem("Obstacle Block"); // type 0
    dropdown->addItem("Free Block"); // type 1
    dropdown->addItem("Node Block"); // type 2
    dropdown->addItem("Origin Block"); // type 3
    //dropdown->addItem("Path Block (Not used)"); // type 4
    
    rightLayout->addWidget(dropdown);

    resizeButton = new QPushButton("Resize Grid");
    rightLayout->addWidget(resizeButton);

    // Text fields for minimum and maximum size
    minSizeEdit = new QLineEdit();
    minSizeEdit->setPlaceholderText("Enter minimum size");
    maxSizeEdit = new QLineEdit();
    maxSizeEdit->setPlaceholderText("Enter maximum size");
    rightLayout->addWidget(minSizeEdit);
    rightLayout->addWidget(maxSizeEdit);

    resetButton = new QPushButton("Reset Canvas");
    rightLayout->addWidget(resetButton);

    syncGridButton = new QPushButton("Sync Grid to Canvas (will take a while)");
    rightLayout->addWidget(syncGridButton);

    syncCanvasButton = new QPushButton("Sync Canvas to Grid (will take a while)");
    rightLayout->addWidget(syncCanvasButton);

    printGridButton = new QPushButton("Print Grid");
    rightLayout->addWidget(printGridButton);

    tracePathButton = new QPushButton("Trace Path (Brute Force Solve)");
    rightLayout->addWidget(tracePathButton);

    traceAllPathsButton = new QPushButton("Trace All Paths (LK)");
    rightLayout->addWidget(traceAllPathsButton);

    traceGreedyPathButton = new QPushButton("Trace Greedy Path");
    rightLayout->addWidget(traceGreedyPathButton);

    saveButton = new QPushButton("Save Grid Data");
    rightLayout->addWidget(saveButton);

    openButton = new QPushButton("Open Grid Data");
    rightLayout->addWidget(openButton);


    topSplitter->addWidget(rightPanel);

    // 3. Console at the Bottom
    console = new QTextEdit(this);
    console->setReadOnly(true);
    console->setPlaceholderText("Console output will appear here...");
    mainSplitter->addWidget(console); // Add console to the splitter

    // Set the splitter as the central widget
    setCentralWidget(mainSplitter);

    // Signals and Slots
    connect(dropdown, &QComboBox::currentIndexChanged, this, &QtWidgetsApplication1::handleDropdownChange);

    connect(resizeButton, &QPushButton::clicked, this, &QtWidgetsApplication1::ResizeCanvas);
    connect(resetButton, &QPushButton::clicked, this, &QtWidgetsApplication1::InitCanvas);
    connect(syncGridButton, &QPushButton::clicked, this, &QtWidgetsApplication1::syncGrid);
    connect(syncCanvasButton, &QPushButton::clicked, this, &QtWidgetsApplication1::syncCanvas);
    connect(printGridButton, &QPushButton::clicked, this, &QtWidgetsApplication1::printGrid);
    connect(tracePathButton, &QPushButton::clicked, this, &QtWidgetsApplication1::tracePath);
    connect(traceGreedyPathButton, &QPushButton::clicked, this, &QtWidgetsApplication1::traceGreedyPath);
    connect(traceAllPathsButton, &QPushButton::clicked, this, &QtWidgetsApplication1::traceAllPaths);

    connect(saveButton, &QPushButton::clicked, this, &QtWidgetsApplication1::saveToFile);
    connect(openButton, &QPushButton::clicked, this, &QtWidgetsApplication1::openFromFile);

    // place initial blocks
    InitCanvas();

    myGrid = new Grid(sceneWidth/10, sceneHeight/10);
    mapView->getGrid();

    //myGrid->setCell(2, 2, 0);
    //Qstring gridoutput = myGrid->printGrid();

    //Cell& origin = myGrid.getCell(0, 0);
    //origin.f = 0;
    //origin.g = 0;
    //origin.h = 0;

    //myGrid.origin = &origin;
    //myGrid.finish = &myGrid.getCell(5, 8); // why is heuristic distance not working??

    //QString text = myGrid.printGrid();
    //WriteConsole(text);
    //std::vector<Cell*> cells = myGrid.getAdjacentCells(0,0); // update adjacent cells and return them so we can choose the next one to visit

    //std::vector<Cell*> path = myGrid.TracePath();
}

QtWidgetsApplication1::~QtWidgetsApplication1()
{
    // Clear instance on destruction
    instance = nullptr;
}
