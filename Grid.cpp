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

        // if we're replacing an old node with something else ( we don't have to check if they're the
        // same because we already did that above)
        if (oldtype == 2) {
            // erase-remove idiom
            nodes.erase(std::remove(nodes.begin(), nodes.end(), oldcell), nodes.end());
        }

        if ((oldtype == 3)) {
            // erase-remove idiom
            nodes.erase(std::remove(nodes.begin(), nodes.end(), origin), nodes.end());
            origin = nullptr;
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

    for (int i = 0; i < y; ++i) {
        cells[i].resize(x);  // Only resizes if more columns are needed
        for (int j = 0; j < x; ++j) {
            // Update the coordinates of each cell
            cells[i][j].x = j;
            cells[i][j].y = i;
        }
    }

    _width = x;
    _height = y;
}

// resize the Grid by x (row length) and y (column length)
void Grid::resizeTSPMap(int x) {

    tspMap.resize(x);  // Resize and initialize

    for (int i = 0; i < x; ++i) {
        tspMap[i].resize(x);  // Only resizes if more columns are needed
        for (int j = 0; j < x; ++j) {
            tspMap[i][j] = 0; // reset values to zero for our next iteration
        }
    }
}

void Grid::clearPathMap() {
    // Iterate over each entry in the outer QHash
    auto iter = pathMap.begin();
    while (iter != pathMap.end()) {
        // iter.value() is the inner QHash
        QHash<QPoint, PathInfo*>& innerHash = iter.value();

        // Iterate over each entry in the inner QHash and delete the PathInfo pointers
        auto innerIter = innerHash.begin();
        while (innerIter != innerHash.end()) {
            delete innerIter.value();  // Delete the PathInfo object pointed to by the pointer
            innerIter++;
        }

        // Clear the inner QHash
        innerHash.clear();

        // Move to the next entry in the outer QHash
        iter++;
    }

    // Clear the outer QHash
    pathMap.clear();
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
        //if (A->type == 1) { // if free
        //    A->type = 4; // change to traverse type ( this is for debug only, remove later)
        //}
        path.push_back(A);
        A = A->fastestneighbor; // functionally a linked list
    }

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

// the structure of Pathmap is a nested hashmap. QHash<QPoint, QHash<QPoint, PathInfo*>> pathMap;
// For each point, there is a nested hash that contains all the distances to all the nodes and their paths.
// we later use this to solve the TSP problem using a distance matrix
void Grid::addPath(const QPoint& start, const QPoint& finish, const std::vector<Cell*>& path)
{
    
    // check if our path Map has our start as the key
    if (pathMap.contains(start)) {
        QHash<QPoint, PathInfo*>& innerHash = pathMap[start]; // 
        if (innerHash.contains(finish)) {
            return; // already here, cancel
        }
    }

    float cost = path.back()->f;  // Assuming path is non-empty, grab f value of the last item in the vector
    PathInfo* _path = new PathInfo(cost, path);
    std::vector<Cell*> reversedPath = path;
    std::reverse(reversedPath.begin(), reversedPath.end());
    PathInfo* _pathReversed = new PathInfo(cost, reversedPath);

    pathMap[start][finish] = _path;
    pathMap[finish][start] = _pathReversed; // Should be _pathReversed instead of _path
}

// returns the pathinfo struct for a given start and end point
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

    int nodesize = nodes.size();
    resizeTSPMap(nodesize);
    clearPathMap();
    //pathMap.clear(); // MEMORY LEAK ALERT
    
    // Loop through each element in the vector
    for (size_t start = 0; start < nodesize; ++start) {

        // Print the starting number followed by a dash
        Cell* temp_start = nodes[start];

        // Loop to print the remaining numbers after the current start
        for (size_t end = start + 1; end < nodesize; ++end) {
            Cell* temp_end = nodes[end];

            // mutex code just in case we get some desync issues
            // mutex.lock();   // Lock the mutex
            // this can be greatly improved if we memoize the reverse route
            std::vector<Cell*> path = TracePath(temp_start->x, temp_start->y, temp_end->x, temp_end->y);
            // mutex.unlock(); // Unlock the mutex
            QPoint startP = QPoint(temp_start->x, temp_start->y);
            QPoint endP = QPoint(temp_end->x, temp_end->y);

            float cost = path.back()->f;  // Assuming path is non-empty, grab f value of the last item in the vector
            tspMap[start][end] = cost;
            tspMap[end][start] = cost;

            addPath(startP, endP, path);
            resetPathingData(); // we're done with this path so reset everything and go again
        }
    }

    return;
}

std::pair<int, std::vector<int>> Grid::TSPSolve_heldKarp() // this shouldn't be void later, make it return the path.
{
    // Held-Karp algorithm is useful because you set predecessors which lets you create a TSP from a specific point.
    // We pick an "arbitrary" point as zero and we rotate it around later.
    // NOT REALLY MY ORIGINAL CODE.
    // Credit to this page: https://compgeek.co.in/held-karp-algorithm-for-tsp/#:~:text=The%20algorithm%20uses%20a%20dynamic,(n%20*%202n).
    addAllPaths(); // this initializes the TSP Map and the all the paths

    //std::vector<std::vector<float>>* ptrTspMap = &tspMap;  // Pointer to tspMap

    int n = tspMap.size();

    // this would not work for more than 32 cities but it's okay because atp the computation is massive.
    // TODO: if n is greater than 32 print ("too many cities for this algorithm")
    int totalSubsets = 1 << n; // Bitmask representing all cities visited

    // Create a dynamic programming table table where dp[mask][i] is the min cost to visit all cities in 'mask' ending in city 'i'
    // initialize all values to -1 representing infinity or NaN
    std::vector<std::vector<double>> dp(totalSubsets, std::vector<double>(n, -1));
    std::vector<std::vector<int>> pred(totalSubsets, std::vector<int>(n, -1)); // store predecessors to recreate a path
    // Base case: cost to visit only the starting city (0) and end there
    dp[1][0] = 0;

    // Iterate over all subsets of cities
    for (int mask = 1; mask < totalSubsets; mask++) {
        for (int u = 0; u < n; u++) {
            if (mask & (1 << u) && dp[mask][u] != -1) {
                for (int v = 0; v < n; v++) {
                    if (!(mask & (1 << v))) {
                        int newMask = mask | (1 << v);
                        double newCost = dp[mask][u] + tspMap[u][v];
                        if (dp[newMask][v] == -1 || newCost < dp[newMask][v]) {
                            dp[newMask][v] = newCost;   // this is our new minimum cost to this combination of points
                            pred[newMask][v] = u;       // set parent / predecessor
                        }
                    }
                }
            }
        }
    }

    // Find the minimum cost to complete the tour
    double minCost = -1;
    std::vector<int> path;
    int last = -1;
    for (int i = 1; i < n; i++) {
        double cost = dp[totalSubsets - 1][i] + tspMap[i][0];
        if (minCost == -1 || cost < minCost) {
            minCost = cost;
            last = i;
        }
    }

    // Reconstruct the path
    if (last != -1) {
        path.push_back(0); // start from the starting city
        int currentMask = totalSubsets - 1;
        while (last != 0) {
            path.push_back(last);
            int temp = last;
            last = pred[currentMask][last];
            currentMask ^= (1 << temp);
        }
        path.push_back(0); // return to the start
        std::reverse(path.begin(), path.end());
    }

    // we now have our path in index form, but we still need the actual real path
    // we can't output this until we can print to the console so we keep it as this simple return as to not screw anything up.


    return {minCost, path};
}