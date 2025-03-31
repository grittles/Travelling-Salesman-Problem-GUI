#include "GUI.h"



// Define the static instance (This is for linking to work)
QtWidgetsApplication1* QtWidgetsApplication1::instance = nullptr;

void QtWidgetsApplication1::handleDropdownChange(int index)
{
    
    selectedDropType = index;

    QString selectedType = QString("Selected Type %1").arg(index + 1);
    WriteConsole(selectedType);
}

void QtWidgetsApplication1::handleDropdownChangeAlgo(int index)
{

    selectedAlgorithm = index;

    //int index = dropdown_Algorithm->currentIndex();
    QString text = dropdown_Algorithm->currentText();

    QString selectedType = QString("Selected Algorithm: %1").arg(text);
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

void QtWidgetsApplication1::startMainTimer()
{
    startTimer();
    //ptrManager->isRunning = true;
}

void QtWidgetsApplication1::stopMainTimer()
{
    stopTimer();
    //ptrManager->isRunning = false;
}



void QtWidgetsApplication1::openFromFile()
{
    QString word = QString("Opening File");
    WriteConsole(word);

    myGrid->openFile(_parent);

    syncGrid();
}

void QtWidgetsApplication1::randomBlock()
{
    QString word = QString("Placing random block...");
    WriteConsole(word);

    mapView->PlaceRandomBlock(2);
}

void QtWidgetsApplication1::cpumemStats()
{
    QString word = QString("Gathering Other Statistics...");
    WriteConsole(word);

    mapView->resetCanvas(sceneWidth, sceneHeight, bSize);
    syncCanvas();

    myGrid->Solve100_Euclidean();


    mapView->resetCanvas(sceneWidth, sceneHeight, bSize);


}

void QtWidgetsApplication1::timeStats()
{
    QString word = QString("Gathering Time Statistics...");
    WriteConsole(word);

    mapView->resetCanvas(sceneWidth, sceneHeight, bSize);
    syncCanvas();
    
    myGrid->Solve100();

    mapView->resetCanvas(sceneWidth, sceneHeight, bSize);
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

    //QString text = myGrid->PrintPath(path);
    //WriteConsole(text);

    //dropdown_Algorithm->addItem("Greedy Algorithm"); // type 0
    //dropdown_Algorithm->addItem("2-opt Brute Force"); // type 1
    //dropdown_Algorithm->addItem("Brute Force Held-Karp"); // type 2
    //dropdown_Algorithm->addItem("Genetic Algorithm"); // type 3
    //dropdown_Algorithm->addItem("Hacky LKH (not accurate)"); // type 4
    std::pair<int, std::vector<Cell*>> result;

    auto start = std::chrono::high_resolution_clock::now();
    switch (selectedAlgorithm) {
        case 0:
            result = myGrid->TSPSolve_Greedy();
            break;

        case 1:
            result = myGrid->TSPSolve_LK();
            break;

        case 2:
            result = myGrid->TSPSolve_heldKarp();
            break;

        case 3:
            result = myGrid->TSPSolve_Genetic();
            break;

        case 4:
            result = myGrid->TSPSolve_LK2();
            break;

        default:
            QString text = QString("Something is wrong");
            WriteConsole(text);
            break;

    }

    auto end = std::chrono::high_resolution_clock::now();
    int distance = result.first;

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

    dropdown_BlockType = new QComboBox(this);
    dropdown_BlockType->addItem("Obstacle Block"); // type 0
    dropdown_BlockType->addItem("Free Block"); // type 1
    dropdown_BlockType->addItem("Node Block"); // type 2
    dropdown_BlockType->addItem("Origin Block"); // type 3
    //dropdown_BlockType->addItem("Path Block (Not used)"); // type 4
    
    rightLayout->addWidget(dropdown_BlockType);

    dropdown_Algorithm = new QComboBox(this);
    dropdown_Algorithm->addItem("Greedy Algorithm"); // type 0
    dropdown_Algorithm->addItem("2-opt Brute Force"); // type 1
    dropdown_Algorithm->addItem("Brute Force Held-Karp"); // type 2
    dropdown_Algorithm->addItem("Genetic Algorithm"); // type 3
    dropdown_Algorithm->addItem("Hacky LKH (not accurate)"); // type 4
    
    
    rightLayout->addWidget(dropdown_Algorithm);

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

    //printGridButton = new QPushButton("Print Grid");
    //rightLayout->addWidget(printGridButton);

    tracePathButton = new QPushButton("Trace Path");
    rightLayout->addWidget(tracePathButton);

    saveButton = new QPushButton("Save Grid Data");
    rightLayout->addWidget(saveButton);

    openButton = new QPushButton("Open Grid Data");
    rightLayout->addWidget(openButton);

    randomBlockButton = new QPushButton("Generate Random Block");
    rightLayout->addWidget(randomBlockButton);

    statsButton = new QPushButton("Gather Mem Stats (100 random nodes)");
    rightLayout->addWidget(statsButton);

    statsButton2 = new QPushButton("Gather Time Stats (100 random nodes)");
    rightLayout->addWidget(statsButton2);

    topSplitter->addWidget(rightPanel);

    // 3. Console at the Bottom
    console = new QTextEdit(this);
    console->setReadOnly(true);
    console->setPlaceholderText("Console output will appear here...");
    mainSplitter->addWidget(console); // Add console to the splitter

    // Set the splitter as the central widget
    setCentralWidget(mainSplitter);

    // Signals and Slots
    connect(dropdown_BlockType, &QComboBox::currentIndexChanged, this, &QtWidgetsApplication1::handleDropdownChange);
    connect(dropdown_Algorithm, &QComboBox::currentIndexChanged, this, &QtWidgetsApplication1::handleDropdownChangeAlgo);

    connect(resizeButton, &QPushButton::clicked, this, &QtWidgetsApplication1::ResizeCanvas);
    connect(resetButton, &QPushButton::clicked, this, &QtWidgetsApplication1::InitCanvas);
    //connect(syncGridButton, &QPushButton::clicked, this, &QtWidgetsApplication1::syncGrid);
    //connect(syncCanvasButton, &QPushButton::clicked, this, &QtWidgetsApplication1::syncCanvas);
    //connect(printGridButton, &QPushButton::clicked, this, &QtWidgetsApplication1::printGrid);
    connect(tracePathButton, &QPushButton::clicked, this, &QtWidgetsApplication1::tracePath);
    //connect(traceGreedyPathButton, &QPushButton::clicked, this, &QtWidgetsApplication1::traceGreedyPath);
    //connect(traceAllPathsButton, &QPushButton::clicked, this, &QtWidgetsApplication1::traceAllPaths);
    //connect(traceGeneticButton, &QPushButton::clicked, this, &QtWidgetsApplication1::traceTSPPath);

    connect(saveButton, &QPushButton::clicked, this, &QtWidgetsApplication1::saveToFile);
    connect(openButton, &QPushButton::clicked, this, &QtWidgetsApplication1::openFromFile);

    connect(randomBlockButton, &QPushButton::clicked, this, &QtWidgetsApplication1::randomBlock);

    connect(statsButton, &QPushButton::clicked, this, &QtWidgetsApplication1::cpumemStats);
    connect(statsButton2, &QPushButton::clicked, this, &QtWidgetsApplication1::timeStats);

    // place initial blocks
    InitCanvas();

    myGrid = new Grid(sceneWidth/10, sceneHeight/10);
    //myGrid->getParent(this);
    mapView->getGrid();
    startTimer();

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
