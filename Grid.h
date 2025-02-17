// Grid.h
#ifndef GRID_H
#define GRID_H

#include <vector>
#include <QString>
#include <queue>

struct Cell {
    uint8_t type = 1;               // Default initialized to 1
    uint8_t neighbors = 0b00000000; // neighbors mask. This determines which neighbors can be visited. Default initialized to 0
    bool foundneighbors = false;
    float f = -1.0;                 // f distance, simply g + h
    float g = -1.0;                 // g distance, distance from start/origin node
    float h = -1.0;                 // heuristic distance from finish. Default initialized to -1. MAKE SURE TO RESET G AND H FOR EVERY
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
    float* priority;  // Pointer to the float priority (f value)
    Cell* cell;       // Pointer to the Cell object

    // Constructor for easier initialization
    CellEntry(float* p, Cell* c) : priority(p), cell(c) {}
};

struct Compare {
    bool operator()(const CellEntry& a, const CellEntry& b) {
        return *a.priority > *b.priority;  // Dereference the pointer to compare values
    }
};

typedef std::priority_queue<CellEntry, std::vector<CellEntry>, Compare> Queue;

class Grid {
private:
    int _width;
    int _height;

    std::vector<std::vector<Cell>> cells;  // 2D vector of Cells
    std::vector<Cell*> nodes;  // vector of Cell pointers; this one is our node network

    // use these offsets to find neighbors
    std::vector<std::pair<int, int>> offsets = {
        {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}
    }; // Order: N(0), NE(1), E(2), SE(3), S(4), SW(5), W(6), NW(7)

public:
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
    uint8_t getNeighbors(int x, int y, Cell* tempcell = nullptr);
    std::vector<Cell*> getAdjacentCells(Cell* point = nullptr);
    std::vector<Cell*> TracePath(int x, int y, int targetx, int targety);
    void resetPath();
    void fullResetPath();

public slots:
    void resizeGrid(int x, int y);
    
    void setCell(int x, int y, int newtype);
    Cell& getCell(int x, int y);

};

#endif // GRID_H
