// Grid.h
#ifndef GRID_H
#define GRID_H

#include <vector>
#include <map>
#include <QString>
#include <QtCore/QPoint.h>
#include <QtCore/QHash.h>
#include <queue>
#include "LKMatrix.h"
#include "FileHandler.h"
#include <QFile>

struct Cell {
    uint8_t type = 1;               // Default initialized to 1
    uint8_t neighbors = 0b00000000; // neighbors mask. This determines which neighbors can be visited. Default initialized to 0
    bool foundneighbors = false;
    double f = -1.0;                 // f distance, simply g + h
    double g = -1.0;                 // g distance, distance from start/origin node
    double h = -1.0;                 // heuristic distance from finish. Default initialized to -1. MAKE SURE TO RESET G AND H FOR EVERY
                                    // CELL AFTER EACH USE OF THE TRAVERSAL ALGORITHM!
    int x = 0;                      // x pos
    int y = 0;                      // y pos

    bool visiting = false;
    bool visited = false;

    Cell* fastestneighbor = nullptr; // this is the cell we will trace back to the start to determine our path

    // Constructors can be defined inline in the struct definition
    Cell() = default;  // Default constructor
    Cell(int t, int x, int y) : type(t), x(x), y(y) {}  // Constructor with parameters

    void resetTraversalProperties(bool resetAll = false) {
        // we do want to keep our foundneighbors boolean and neighbors mask, just reset the 
        // info related to start and finish
        f = -1.0;
        g = -1.0;
        h = -1.0;
        visiting = false;
        visited = false;
        fastestneighbor = nullptr;

        if (resetAll == true) {
            foundneighbors = false;

            if (type == 4) {
                type = 1;
            }
        }
    }
};

// --------------------------------------//
// Priority queue implementation below
// --------------------------------------//
struct CellEntry {
    double* priority;  // Pointer to the float priority (f value)
    Cell* cell;       // Pointer to the Cell object

    // Constructor for easier initialization
    CellEntry(double* p, Cell* c) : priority(p), cell(c) {}
};

struct Compare {
    bool operator()(const CellEntry& a, const CellEntry& b) {
        return *a.priority > *b.priority;  // Dereference the pointer to compare values
    }
};

typedef std::priority_queue<CellEntry, std::vector<CellEntry>, Compare> Queue;

// PathInfo struct we use to store distance and the actual path
struct PathInfo {
    double distance = 0;
    std::vector<Cell*> path;

    PathInfo() = default;  // Default constructor
    PathInfo(double d, std::vector<Cell*> p) : distance(d), path(p) {}
};

double calculate_h_score(const int x1, const int y1, const int x2, const int y2);
std::pair<int, std::vector<int>> _TSPSolve_heldKarp(std::vector<std::vector<double>> e_matrix);

std::pair<double, std::vector<int>> _LK_Route(std::vector<QPoint>* coords, std::vector<int>* ids, std::vector<std::vector<double>>* E_DistMatrix);


class Grid {
private:
    int _width;
    int _height;

    std::vector<std::vector<Cell>> cells;  // 2D vector of Cells

    // these 3 are all we will ever need for any TSP algorithm
    std::vector<std::vector<double>> tspMap;  // 2D vector of floats (euclidean distance matrix baby)
    std::vector<QPoint> _points;  // 2D vector of Cells
    std::vector<int> _currentorder; // this is all we need for any TSP algorithm

    std::vector<Cell*> nodes;  // vector of Cell pointers; this one is our node network
    std::vector<std::pair<int, QPoint>> nodeIndex;  // vector of Cell pointers; this one is our node network
    QHash<QPoint, QHash<QPoint, PathInfo*>> pathMap;

    // use these offsets to find neighbors
    std::vector<std::pair<int, int>> offsets = {
        {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}
    }; // Order: N(0), NE(1), E(2), SE(3), S(4), SW(5), W(6), NW(7)

public:

    std::vector<Cell*>* getNodes() {
        return &nodes;
    }
    
    std::vector<std::vector<Cell>>* getGrid() {
        return &cells;
    }

    // Inline constructor within the class definition
    Grid(int width, int height) {
        _width = width;
        _height = height;

        cells.resize(height, std::vector<Cell>(width));  // Resize and initialize

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                cells[y][x] = Cell(1, x, y);  // Initialize each cell with type 1 (free) and its coordinates
            }
        }
    }

    int min_int(int a, int b) {
        // If either number is -1, return the other number (which is not -1)
        if (a == -1) return b; // If a is -1 (infinity), return b
        else if (b == -1) return a; // If b is -1 (infinity), return a

        // If neither is -1, return the smaller number
        return (a < b) ? a : b;
    }

    double min_float(double a, double b) {
        // If either number is -1, return the other number (which is not -1)
        if (a == -1) return b; // If a is -1 (infinity), return b
        else if (b == -1) return a; // If b is -1 (infinity), return a

        // If neither is -1, return the smaller number
        return (a < b) ? a : b;
    }


    // Get grid dimensions
    int getWidth() const { return _width; }
    int getHeight() const { return _height; }

    // we differentiate origin from start so we can traverse multiple node to node paths with the same grid
    Cell* origin = nullptr; // idk why im using a pointer instead of coordinates, should be faster maybe???
    Cell* start = nullptr;
    Cell* finish = nullptr;

    // Declarations only; definitions will be in Grid.cpp
    QString printGrid() const;
    QString PrintPath(std::vector<Cell*> cells) const;
    bool saveFile(QWidget* parent) const;
    bool openFile(QWidget* parent);

    uint8_t getNeighbors(int x, int y, Cell* tempcell = nullptr);
    std::vector<Cell*> getAdjacentCells(Cell* point = nullptr);
    std::vector<Cell*> TracePath(int x, int y, int targetx, int targety);
    void resetPathingData();
    void fullResetPath();

    void addPath(const QPoint& start, const QPoint& finish, const std::vector<Cell*>& path);

    PathInfo* getPath(const QPoint& start, const QPoint& finish);

    bool addAllPaths();

    std::pair<int, std::vector<Cell*>> TSPSolve_heldKarp();
    std::pair<int, std::vector<Cell*>> TSPSolve_LK();
    std::pair<int, std::vector<Cell*>> TSPSolve_Greedy();

    std::vector<Cell*> fixPath(std::vector<int> tour);

    

public slots:
    void resizeGrid(int x, int y);

    void resizeTSPMap(int x);

    void clearPathMap();
    
    void setCell(int x, int y, int newtype);
    Cell& getCell(int x, int y);

};
#endif // GRID_H
