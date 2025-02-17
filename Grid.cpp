// Grid.cpp
#include "Grid.h"

// find h score. Recall final score f = g + h
float calculate_h_score(const int x1, const int y1, const int x2, const int y2)
{
	float distance_x = abs(x1 - x2);
	float distance_y = abs(y1 - y2);

	//This is the minimal distance it takes to move from cell_0 to cell_1 if we move in 8 directions and ignore every obstacle.
	//I don't recommend using other types of distance calculations because then Astar doesn't find the shortest path.
	return std::max(distance_x, distance_y) + std::min(distance_x, distance_y) * (sqrt(2) - 1);
}

QString Grid::printGrid() const {
    QString output;  // Use QString

    for (const auto& row : cells) {
        for (const auto& cell : row) {
            output += QString("(%0, %1) ").arg(cell.type).arg(cell.f);
        }
        output += "\n";  // Newline for each row
    }
    return output;
}


QString Grid::PrintPath(std::vector<Cell*> cells) const {
    QString output;  // Use QString
    int sz = cells.size();
    if (sz == 0) {
        return QString("Could not produce a path");
    }
    for (size_t i = 0; i < sz -1; ++i) {
        output += QString("[%0,%1]->").arg(cells[i]->x).arg(cells[i]->y);
    }
    output += QString("[%0,%1]\n").arg(cells[sz - 1]->x).arg(cells[sz -1]->y);

    return output;
}

void Grid::setCell(int x, int y, int newtype) { // couldn't get this to work as a prototype without errors so the whole function is here now

    // make sure we're selecting inside bounds so we don't get memory errors
    if (x >= 0 && x < _width && y >= 0 && y < _height) {

        Cell* oldcell = &cells[y][x];
        int oldtype = oldcell->type;

        // same type? Just return
        if (oldtype == newtype) {
            return;
        }

        oldcell->type = newtype; // update cell
        //oldcell->x = x; // for some reason my constructor is messing up when i resize.
        //oldcell->y = y; // update cell

        // if we're replacing an old node with something else ( we don't have to check if they're the
        // same because we already did that above)
        if (oldtype == 2) {
            // erase-remove idiom
            nodes.erase(std::remove(nodes.begin(), nodes.end(), oldcell), nodes.end());
        }

        // if we're adding a new origin
        if (newtype == 3) {
            if (origin != nullptr) {
                nodes.erase(std::remove(nodes.begin(), nodes.end(), origin), nodes.end());
                origin->type = 1; // replace old type as 1 (free)
            }
            origin = oldcell; // replace the pointer of origin with oldcell pointer
            nodes.push_back(origin);

        } // this won't execute if (oldtype == 2) and newtype == 2 because we checked for the same type earlier.
        else if (newtype == 2) {
            // we have a new node so push it on top of our nodes vector
            nodes.push_back(oldcell);

        }
    }
}


Cell& Grid::getCell(int x, int y) {
    return cells[y][x];
}

// resize the Grid by x (row length) and y (column length)
void Grid::resizeGrid(int x, int y) {
    cells.resize(y);  // Resize and initialize

    for (int i = 0; i < cells.size(); ++i) {
        cells[i].resize(x);  // Only resize if more columns are needed
        for (int j = 0; j < cells[i].size(); ++j) {
            // Update the coordinates of each cell
            cells[i][j].x = j;
            cells[i][j].y = i;
        }
    }


    /*for (auto& row : cells) {
        row.resize(x);
    }*/

    _width = x;
    _height = y;
}

uint8_t Grid::getNeighbors(int x, int y, Cell* tempcell) {
    
    // we might visit cells multiple times, don't repeat finding neighbors for visited cells.
    if (tempcell == nullptr)
        tempcell = &cells[y][x];

    uint8_t status = 0b11111111;  // All neighbors initially assumed valid

    // We're using bitmasking to determine which neighbors are which.
    // Order: N(0), NE(1), E(2), SE(3), S(4), SW(5), W(6), NW(7)

    // Clear bits using bitwise AND and NOT. to get bitwise nand
    if (y == 0) status &= ~(0b10000011);  // Clear NW, N, NE 
    else if (y == getHeight() - 1) status &= ~(0b00111000);  // Clear SW, S, SE

    if (x == 0) status &= ~(0b11100000);  // Clear NW, W, SW
    else if (x == getWidth() - 1) status &= ~(0b00001110);  // Clear E, NE, SE

    // Check traversability for N, E, S, W
    for (int i = 0; i < 8; i += 2) {

        if (status & (1 << i)) {

            int nx = x + offsets[i].first;
            int ny = y + offsets[i].second;

            if (cells[ny][nx].type == 0) // 0 type block means untraversable, everything else is traversable.
                status &= ~(1 << i);  // Mask as untraversable if not traversable
        }
    }

    // Conditional check for diagonals
    for (int i = 1; i < offsets.size(); i += 2) {  // Only diagonals: NE, SE, SW, NW
        if (status & (1 << i)) {

            int nx = x + offsets[i].first;
            int ny = y + offsets[i].second;

            // Check if both immediate neighbors are unblocked
            if (!((status & (1 << (i - 1))) || (status & (1 << ((i + 1) % 8))))) {
                status &= ~(1 << i);  // Mask diagonal if both immediate neighbors are blocked
            }
            else if (cells[ny][nx].type == 0) // 0 means untraversable, everything else is traversable.
                status &= ~(1 << i);  // Mask as untraversable if not traversable
        }
    }

    tempcell->neighbors = status;
    tempcell->foundneighbors = true;

    return status;
}

// Finds adjacent cells and their distances (g values)
// in another function we will use this information to determine where to travel.
std::vector<Cell*> Grid::getAdjacentCells(Cell* point) // replace targetx and targety later when we implement targets
{
    std::vector<Cell*> neighborcells;
    if (point == nullptr) return {};

    int x = point->x;
    int y = point->y;

    int targetx = finish->x;
    int targety = finish->y;

    uint8_t status = point->neighbors;

    if (point->foundneighbors == false) {
        status = Grid::getNeighbors(x, y, point);
    }

    for (int i = 0; i < 8; i += 1) {

        if (status & (1 << i)) {

            int nx = x + offsets[i].first;
            int ny = y + offsets[i].second;

            Cell* tempcell = &cells[ny][nx];
            if (tempcell->visited) {
                continue;
            }

            // first calculate g
            float oldf = tempcell->f;
            float oldg = tempcell->g;
            float oldh = tempcell->h;
            float tempg = point->g;

            if (i & 1) // odd numbers are corners. LSB being 1 indicates an odd number.
                tempg += sqrt(2);
            else
                tempg += 1;

            if (oldg > tempg || oldg < 0)
                tempcell->g = tempg;

            if (oldh < 0)
                oldh = calculate_h_score(nx, ny, targetx, targety);
                tempcell->h = oldh;

            float tempf = oldh + tempg;

            if (oldf < 0 || tempf < oldf) {
                tempcell->f = tempf;
                tempcell->fastestneighbor = point;
            }

            neighborcells.push_back(tempcell);
        }
    }

    return neighborcells;
}

std::vector<Cell*> Grid::TracePath(int x, int y, int targetx, int targety) // replace targetx and targety later when we implement targets
{
    // we aren't concerned with type. So any position can be set to the start, even a block type (so long as it's actually possible to traverse)
    // these are members of the class
    start = &cells[y][x];
    finish = &cells[targety][targetx];

    if (start == nullptr || finish == nullptr) {
        return {};  // Return an empty vector to indicate an error or invalid input
    }

    start->f = 0;
    start->g = 0;
    start->h = 0;

    Queue toVisit;

    Cell* A = start;
    A->visiting = true;

    while (!(A == finish)) {
        A->visited = true;
        std::vector<Cell*> neighborcells = getAdjacentCells(A);
        for (Cell* c : neighborcells) {
            if (c->visiting == false) {
                c->visiting = true;
                toVisit.push(CellEntry(&c->f, c));
            }    
        }

        // choose which to visit next from our priority queue
        if (!toVisit.empty()) {
            A = toVisit.top().cell;
            toVisit.pop();
        }
        else
            return {}; // return no path. We aren't worried about this code executing because A == finish before toVisit is empty
    }

    // our path starts from end and traces back to the start.
    // At the end we reverse it to know what the proper order is.
    std::vector<Cell*> path;
    while (!(A == nullptr)) {
        if (A->type == 1) { // if free
            A->type = 4; // change to traverse type ( this is for debug only, remove later)
        }
        path.push_back(A);
        A = A->fastestneighbor; // functionally a linked list
    }
    //path.push_back(A);
    std::reverse(path.begin(), path.end());

    start = nullptr;
    finish = nullptr;
    //resetPathingData(); // we're done with this path so reset everything

    return path;
}

void Grid::resetPathingData() {
    for (auto& row : cells) {
        for (auto& cell : row) {
            if (cell.visiting == true) {
                cell.resetTraversalProperties();
            }
        }
    }
}

void Grid::fullResetPath()
{
    for (auto& row : cells) {
        for (auto& cell : row) {
            cell.resetTraversalProperties(true);
        }
    }
}

void Grid::addPath(const QPoint& start, const QPoint& finish, const std::vector<Cell*>& path)
{

    if (pathMap.contains(start)) {
        QHash<QPoint, PathInfo*>& innerHash = pathMap[start];
        if (innerHash.contains(finish)) {
            return; // already here, cancel
        }
    }

    float cost = path.back()->f;  // Assuming path is non-empty
    PathInfo* _path = new PathInfo(cost, path);
    std::vector<Cell*> reversedPath = path;
    std::reverse(reversedPath.begin(), reversedPath.end());
    PathInfo* _pathReversed = new PathInfo(cost, reversedPath);

    pathMap[start][finish] = _path;
    pathMap[finish][start] = _pathReversed; // Should be _pathReversed instead of _path
}

PathInfo* Grid::getPath(const QPoint& start, const QPoint& finish)
{
    // Check if the start point exists in the outer hash
    if (pathMap.contains(start)) {
        // Check if the finish point exists in the inner hash for the given start point
        QHash<QPoint, PathInfo*>& innerHash = pathMap[start];
        if (innerHash.contains(finish)) {
            return innerHash[finish];
        }
    }
    return nullptr;
}

void Grid::addAllPaths()
{   
    /*if (origin != nullptr && std::find(nodes.begin(), nodes.end(), origin) == nodes.end()) {
        nodes.push_back(origin);
    }*/

    
    // Loop through each element in the vector
    for (size_t start = 0; start < nodes.size(); ++start) {

        // Print the starting number followed by a dash
        Cell* temp_start = nodes[start];

        // Loop to print the remaining numbers after the current start
        for (size_t end = start + 1; end < nodes.size(); ++end) {
            Cell* temp_end = nodes[end];

            // mutex code just in case we get some desync issues
            // mutex.lock();   // Lock the mutex
            std::vector<Cell*> path = TracePath(temp_start->x, temp_start->y, temp_end->x, temp_end->y);
            // mutex.unlock(); // Unlock the mutex
            QPoint startP = QPoint(temp_start->x, temp_start->y);
            QPoint endP = QPoint(temp_end->x, temp_end->y);
            addPath(startP, endP, path);
            resetPathingData(); // we're done with this path so reset everything and go again
        }
    }

    //auto it = std::remove(nodes.begin(), nodes.end(), origin);
    //nodes.erase(it, nodes.end());
}