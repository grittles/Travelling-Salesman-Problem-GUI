#include "GUI.h"

int selectedDropType = 0;

// Define the static instance (This is required for linking to work)
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

void QtWidgetsApplication1::addObject()
{

    QString word = QString("Adding an object");
    WriteConsole(word);
}

void QtWidgetsApplication1::deleteObject()
{
    QString word = QString("Deleting an object");
    WriteConsole(word);
}

void QtWidgetsApplication1::ResizeCanvas()
{
    bool okMin, okMax;

    int minSize = minSizeEdit->text().toInt(&okMin);
    int maxSize = maxSizeEdit->text().toInt(&okMax);

    // this could cause some real problems if this bool check fails, beware.
    if (okMin && okMax) {
        ResizeCanvasArgs(minSize, maxSize);
    }
    else {
        WriteConsole("Invalid input for resizing");
    }
}

void QtWidgetsApplication1::ResizeCanvasArgs(int minSize = -1, int maxSize = -1)
{

    // minSize = std::round(static_cast<double>(minSize) / 10)* 10; // round to nearest 10.
    // maxSize = std::round(static_cast<double>(maxSize) / 10) * 10; // round to nearest 10.

    // Assuming you want to resize the scene rect
    // we're just going to assume blocksize is 10 at this point because what else would it be?
    mapScene->setSceneRect(0, 0, minSize*10, maxSize*10);

    sceneWidth = minSize*10;
    sceneHeight = maxSize*10;

    mapView->resizeCanvas();

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
    std::vector<Cell*> path = myGrid->TracePath(0,0,5,8);
    QString text = myGrid->PrintPath(path);
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
    myGrid->addAllPaths();

    auto result = myGrid->TSPSolve_heldKarp();
    auto path = result.second;

    std::vector<Cell*> finalpath;
    std::vector<Cell*>* nodes = myGrid->getNodes();
    for (int city : path) {
        
        finalpath.push_back((*nodes)[city]);

    }

    for (int i = 0; i < finalpath.size() -1; i++) {
        QPoint start = QPoint(finalpath[i]->x, finalpath[i]->y);
        QPoint finish = QPoint(finalpath[i+1]->x, finalpath[i+1]->y);

        PathInfo* pathInfo = myGrid->getPath(start, finish);
        std::vector<Cell*> path = pathInfo->path;


        for (Cell* c : path) {
            if (c->type == 1) { // if free
                c->type = 4; // change to traverse type ( this is for debug only, remove later)
            }
            //myGrid->setCell(c->x,c->y, );
        }

    }

    auto c = myGrid->origin;
    auto it = std::find(finalpath.begin(), finalpath.end(), c);
    if ((it != finalpath.begin()) && (it != finalpath.end())) {
        finalpath.pop_back(); // Remove the last element

        std::rotate(finalpath.begin(), it, finalpath.end());

        finalpath.push_back(c);
    }


    QString text = myGrid->PrintPath(finalpath);

    WriteConsole(text);

    text = QString("origin: [%1, %2]").arg(c->x).arg(c->y);
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
    mapScene->setSceneRect(0, 0, 800, 600);

    //viewWidth = mapView->width();
    //viewHeight = mapView->height();

    //double scaleX = 2;
    //double scaleY = 2;

    //mapView->scale(scaleX, scaleY);

    QRectF sceneRect = mapScene->sceneRect(); // Get the scene rectangle
    sceneWidth = static_cast<int>(std::round(sceneRect.width()));
    sceneHeight = static_cast<int>(std::round(sceneRect.height()));


    topSplitter->addWidget(mapView);

    topSplitter->addWidget(mapView);

    // 2. Right Panel
    rightPanel = new QWidget(this);
    rightLayout = new QVBoxLayout(rightPanel);

    dropdown = new QComboBox(this);
    dropdown->addItem("Obstacle Block"); // type 0
    dropdown->addItem("Free Block"); // type 1
    dropdown->addItem("Node Block"); // type 2
    dropdown->addItem("Origin Block"); // type 3
    dropdown->addItem("Path Block (Not used)"); // type 4
    
    rightLayout->addWidget(dropdown);

    addButton = new QPushButton("Add Object");
    rightLayout->addWidget(addButton);

    deleteButton = new QPushButton("Delete Object");
    rightLayout->addWidget(deleteButton);

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

    tracePathButton = new QPushButton("Trace Path");
    rightLayout->addWidget(tracePathButton);

    traceAllPathsButton = new QPushButton("Trace All Paths");
    rightLayout->addWidget(traceAllPathsButton);

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
    connect(addButton, &QPushButton::clicked, this, &QtWidgetsApplication1::addObject);
    connect(deleteButton, &QPushButton::clicked, this, &QtWidgetsApplication1::deleteObject);
    connect(resizeButton, &QPushButton::clicked, this, &QtWidgetsApplication1::ResizeCanvas);
    connect(resetButton, &QPushButton::clicked, this, &QtWidgetsApplication1::InitCanvas);
    connect(syncGridButton, &QPushButton::clicked, this, &QtWidgetsApplication1::syncGrid);
    connect(syncCanvasButton, &QPushButton::clicked, this, &QtWidgetsApplication1::syncCanvas);
    connect(printGridButton, &QPushButton::clicked, this, &QtWidgetsApplication1::printGrid);
    connect(tracePathButton, &QPushButton::clicked, this, &QtWidgetsApplication1::tracePath);
    connect(traceAllPathsButton, &QPushButton::clicked, this, &QtWidgetsApplication1::traceAllPaths);

    // place initial blocks
    InitCanvas();

    myGrid = new Grid(10, 10);
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
