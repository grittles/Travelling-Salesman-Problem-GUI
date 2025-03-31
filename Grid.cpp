// Grid.cpp
#include "Grid.h"
#include "GUI.h"


// Calculate the perpendicular distance from point C(x3, y3) to the line formed by A(x1, y1) and B(x2, y2)
double calculate_d_score(int x1, int y1, int x2, int y2, int x3, int y3)
{
    // Compute the area of triangle ABC using the determinant method
    double area = fabs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0;

    // Calculate the magnitude of the vector AB
    double magnitudeAB = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

    // Compute the perpendicular distance from point C to line AB
    double distance = (2 * area) / magnitudeAB;

    return distance;
}

// find h score. Recall final score f = g + h
// old version SUCKED so we use euclidean instead.
double calculate_h_score(const int x1, const int y1, const int x2, const int y2)
{
    double distance_x = abs(x1 - x2);
    double distance_y = abs(y1 - y2);

    //This is the minimal distance it takes to move from cell_0 to cell_1 if we move in 8 directions and ignore every obstacle.
    //I don't recommend using other types of distance calculations because then Astar doesn't find the shortest path.
    //return std::max(distance_x, distance_y) + std::min(distance_x, distance_y) * (sqrt(2) - 1);
    return sqrt(distance_x * distance_x + distance_y * distance_y);
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
    for (size_t i = 0; i < sz - 1; ++i) {
        output += QString("[%0,%1]->").arg(cells[i]->x).arg(cells[i]->y);
    }
    output += QString("[%0,%1]\n").arg(cells[sz - 1]->x).arg(cells[sz - 1]->y);

    return output;
}

bool Grid::saveFile(QWidget* parent) const {
    char* b = (char*)malloc((_width * (_height + 1)) + 1); // +1 for null terminator and +1 for newline char on every line
    if (b == nullptr) return false; // malloc failed
    
    int strsize = sprintf(b, "%d %d\n", _width, _height); // returns the number of characters written, not including null terminator

    char* p = b + strsize;

    for (int i = 0; i < _height; i++) {
        for (int j = 0; j < _width; j++) {
            *p = '0' + cells[i][j].type; // types range from 0-4 so this will always be valid
            p++;
            strsize += 1;
        }
        *p = '\n'; // add newline after every row
        p++;
        strsize += 1;
    }
    *p = '\0'; // Null-terminate the string
    strsize += 1;

    QFile* file = openFileForWriting(parent);
    if (file == nullptr) {
        delete file; // Clean up
        return false;
    }
    

    //QString qstr = QString::fromUtf8(b); // Convert using UTF-8 encoding.

    //size_t len = strlen(b);
    QByteArray byteArray(b, strsize); // Copies data into QByteArray
    free(b);

    file->write(byteArray);

    file->close();
    delete file;

    return true;
}

bool Grid::openFile(QWidget* parent) {
    QFile* file = openFileForReading(parent);
    if (file == nullptr) return false;

    QTextStream in(file);
    QString line = in.readLine(); // Read the first line containing dimensions, etc.

    QStringList dimensions = line.split(' ');
    if (dimensions.size() != 2) {
        file->close();
        return false; // Error handling in case the format is not as expected
    }

    bool ok1, ok2;
    int width = dimensions.at(0).toInt(&ok1);
    int height = dimensions.at(1).toInt(&ok2);

    if (!ok1 || !ok2) {
        file->close();
        return false; // Error handling in case conversion fails
    }

    resizeGrid(width, height);
    fullResetPath();


    // Read the rest of the data byte by byte
    int x = 0, y = 0;
    while (y < height) {
        QChar ch;
        in >> ch; // Read character by character

        if (ch == '\n') {
            y++;
            x = 0;
            continue;
        }

        // Assuming '0' character offset for grid values
        if (x < width) {
            setCell(x, y, ch.toLatin1() - '0');
            x++;
        }
    }

    file->close();
    delete file;
    return true;
    
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

    if (x == _width && y == _height) {
        return;
    }
    
    // resizing results in dangling pointers so we need to rebuild our nodes. Really kinda unfortunately this is.
    nodes.clear();

    cells.resize(y);  // Resize and initialize

    for (int i = 0; i < y; ++i) {

        cells[i].resize(x);  // Only resizes if more columns are needed
        for (int j = 0; j < x; ++j) {
            // Update the coordinates of each cell
            Cell*item = &cells[i][j];

            item->x = j;
            item->y = i;

            int type = item->type;
            if (type == 2 || type == 3) {
                nodes.push_back(item);
            }
        }
    }

    _width = x;
    _height = y;
}

void Grid::resetGrid() {
    nodes.clear();

    for (int i = 0; i < _height; ++i) {
        for (int j = 0; j < _width; ++j) {
            // Update the coordinates of each cell
            Cell* item = &cells[i][j];

            item->x = j;
            item->y = i;
            item->type = 1;
        }

    }
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

    int startx = start->x;
    int starty = start->y;

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
            double oldf = tempcell->f;
            double oldg = tempcell->g;
            double oldh = tempcell->h;
            double tempg = point->g;

            if (i & 1) // odd numbers are corners. LSB being 1 indicates an odd number.
                tempg += sqrt(2);
            else
                tempg += 1;

            if (oldg > tempg || oldg < 0)
                tempcell->g = tempg;

            if (oldh < 0)
                oldh = calculate_h_score(nx, ny, targetx, targety);
                //double oldd = min_float(calculate_d_score(nx, ny, targetx, targety, startx, starty), oldh/100);

                tempcell->h = oldh;

            double tempf = oldh + tempg;

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
        path.push_back(A);
        A = A->fastestneighbor; // functionally a linked list
    }

    std::reverse(path.begin(), path.end()); // inefficient, change this later

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
        QHash<QPoint, PathInfo*>& innerHash = pathMap[start]; //dereference 
        if (innerHash.contains(finish)) {
            return; // already here, cancel
        }
    }

    double cost = path.back()->f;  // Assuming path is non-empty, grab f value of the last item in the vector
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

// A better name would be Generate Euclidean Distance Matrix. Calculates all Distances to every node for every node.
bool Grid::addAllPaths()
{   

    int nodesize = nodes.size();

    if (nodesize < 2) return false;

    resizeTSPMap(nodesize);
    clearPathMap();
    _points.clear();
    _currentorder.clear();

    //pathMap.clear(); // MEMORY LEAK ALERT
    
    // Loop through each element in the vector
    for (size_t start = 0; start < nodesize; ++start) {

        // Print the starting number followed by a dash
        Cell* temp_start = nodes[start];
        QPoint startP = QPoint(temp_start->x, temp_start->y);

        _points.push_back(startP);
        _currentorder.push_back(start);

        // Loop to print the remaining numbers after the current start
        for (size_t end = start + 1; end < nodesize; ++end) {
            Cell* temp_end = nodes[end];

            // mutex code for any future multithreading implementation
            // mutex.lock();   // Lock the mutex
            // this can be greatly improved if we memoize the reverse route
            std::vector<Cell*> path = TracePath(temp_start->x, temp_start->y, temp_end->x, temp_end->y);

            if (path.size() == 0) {
                return false;
                //tspMap[start][end] = -1;
                //resetPathingData(); // we're done with this path so reset everything and go again
                //continue;
            }
            // mutex.unlock(); // Unlock the mutex
            
            QPoint endP = QPoint(temp_end->x, temp_end->y);
            
            double cost = path.back()->f;  // Assuming path is non-empty, grab f value of the last item in the vector
            tspMap[start][end] = cost;
            tspMap[end][start] = cost;

            addPath(startP, endP, path);
            resetPathingData(); // we're done with this path so reset everything and go again
        }
    }

    return true;
}

std::pair<int, std::vector<Cell*>> Grid::TSPSolve_heldKarp() // this shouldn't be void later, make it return the path.
{
    if (!(addAllPaths())) return {0, {}}; // this initializes the TSP Map and the all the paths

    //std::vector<std::vector<double>>* ptrTspMap = &tspMap;  // Pointer to tspMap

    //int n = tspMap.size();

    auto returnvalue = _TSPSolve_heldKarp(tspMap);
    std::vector<Cell*> finalroute = fixPath(returnvalue.second);

    return { returnvalue.first, finalroute };

}

std::pair<int, std::vector<int>> _TSPSolve_heldKarp(std::vector<std::vector<double>> e_matrix)
{
    // Held-Karp algorithm is useful because you set predecessors which lets you create a TSP from a specific point.
    // We pick an "arbitrary" point as zero and we rotate it around later.
    // NOT REALLY MY ORIGINAL CODE.
    // Credit to this page: https://compgeek.co.in/held-karp-algorithm-for-tsp/#:~:text=The%20algorithm%20uses%20a%20dynamic,(n%20*%202n).

    // this would not work for more than 32 cities but it's okay because atp the computation is massive.
    // TODO: if n is greater than 32 print ("too many cities for this algorithm")

    int n = e_matrix.size();

    if (n < 2) return { 0, {}};

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
                        double newCost = dp[mask][u] + e_matrix[u][v];
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
        double cost = dp[totalSubsets - 1][i] + e_matrix[i][0];
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

    return { minCost, path };
}

std::pair<int, std::vector<int>> _TSPSolve_BruteForce(std::vector<std::vector<double>> e_matrix)
{

    int n = e_matrix.size();

    if (n < 2) return { 0, {} };

    std::vector<int> cities(n);
    std::iota(cities.begin(), cities.end(), 0);  // Fill cities with 0, 1, ..., n-1

    std::vector<int> bestTour;

    double minCost;
    for (int i = 0; i < n; i++) {
        minCost = e_matrix[cities[i]][cities[(i + 1) % n]];
    }

    while (std::next_permutation(cities.begin(), cities.end())) {
        double currentDistance = 0;
        for (int i = 0; i < n; i++) {
            currentDistance += e_matrix[cities[i]][cities[(i + 1) % n]];
        }
        if (currentDistance < minCost) {
            minCost = currentDistance;
            bestTour = cities;
        }
    }

    return { minCost, bestTour };
}



// traces the path and makes them traversal type, then, rotates around the origin to make it the start and end point.
std::vector<Cell*> Grid::fixPath(std::vector<int> tour) // this shouldn't be void later, make it return the path.
{
    int toursize = tour.size();

    if (toursize < 3) return {};

    std::vector<Cell*> finalpath;
    std::vector<Cell*>* nodes = getNodes();

    for (int city : tour) {

        finalpath.push_back((*nodes)[city]);

    }

    for (int i = 0; i < toursize - 1; i++) {
        QPoint start = QPoint(finalpath[i]->x, finalpath[i]->y);
        QPoint finish = QPoint(finalpath[i + 1]->x, finalpath[i + 1]->y);

        PathInfo* pathInfo = getPath(start, finish);
        std::vector<Cell*> path = pathInfo->path;

        for (Cell* c : path) {
            if (c->type == 1) { // if free
                c->type = 4; // change to traverse type
            }
        }
    }

    auto it = std::find(finalpath.begin(), finalpath.end(), origin);
    if ((it != finalpath.begin()) && (it != finalpath.end())) {
        finalpath.pop_back(); // Remove the last element

        std::rotate(finalpath.begin(), it, finalpath.end());

        finalpath.push_back(origin);
    }

    return finalpath;

}

std::pair<int, std::vector<Cell*>> Grid::TSPSolve_LK() // this shouldn't be void later, make it return the path.
{
    if (!(addAllPaths())) return { 0, {} }; // this initializes the TSP Map and the all the paths

    auto returnvalue = _LK_Route(&_points, &_currentorder, &tspMap);
    std::vector<Cell*> finalroute = fixPath(returnvalue.second);

    return { returnvalue.first, finalroute };
}

std::pair<int, std::vector<Cell*>> Grid::TSPSolve_LK2() // this shouldn't be void later, make it return the path.
{
    if (!(addAllPaths())) return { 0, {} }; // this initializes the TSP Map and the all the paths

    auto returnvalue = _LK_Route2(&_points, &_currentorder, &tspMap);
    std::vector<Cell*> finalroute = fixPath(returnvalue.second);

    return { returnvalue.first, finalroute };
}

std::pair<double, std::vector<int>> _LK_Route(std::vector<QPoint>* coords, std::vector<int>* ids, std::vector<std::vector<double>>* E_DistMatrix) {
    
    LK lkroute;
    lkroute.LKMatrix(coords, ids, E_DistMatrix);
    lkroute.optimizeTour();
    double distance = lkroute.getCurrentTourDistance(0);
    std::vector<int> tour = (*lkroute.retrieveTour());

    return { distance, tour };
}

std::pair<double, std::vector<int>> _LK_Route2(std::vector<QPoint>* coords, std::vector<int>* ids, std::vector<std::vector<double>>* E_DistMatrix) {

    LK lkroute;
    lkroute.LKMatrix(coords, ids, E_DistMatrix);
    lkroute.optimizeTour2();
    double distance = lkroute.getCurrentTourDistance(0);
    std::vector<int> tour = (*lkroute.retrieveTour());

    return { distance, tour };
}


std::pair<int, std::vector<Cell*>> Grid::TSPSolve_Genetic() // this shouldn't be void later, make it return the path.
{
    // addallpaths returns false if it can't add all paths
    if (!(addAllPaths())) return { 0, {} }; // this initializes the TSP Map and the all the paths

    auto returnvalue = _Genetic_Route(&_points, &_currentorder, &tspMap);

    std::vector<Cell*> finalroute = fixPath(returnvalue.second);

    return { returnvalue.first, finalroute };
}

std::pair<double, std::vector<int>> _Genetic_Route(std::vector<QPoint>* coords, std::vector<int>* ids, std::vector<std::vector<double>>* E_DistMatrix) {

    TSPGenetic geneticroute;
    geneticroute.Create(coords, ids, E_DistMatrix);
    geneticroute.optimizeTour(1000);
    double distance = geneticroute.getCurrentTourDistance();
    std::vector<int> tour = (*geneticroute.retrieveTour());
    return { distance, tour };
}


std::pair<int, std::vector<Cell*>> Grid::TSPSolve_Greedy() // this shouldn't be void later, make it return the path.
{
    if (!(addAllPaths())) return { 0, {} }; // this initializes the TSP Map and the all the paths

    //std::vector<std::vector<double>>* ptrTspMap = &tspMap;  // Pointer to tspMap

    //int n = tspMap.size();

    auto returnvalue = greedyRoute(&tspMap);
    std::vector<Cell*> finalroute = fixPath(returnvalue.second);

    return { returnvalue.first, finalroute };
}

void Grid::Solve100() // this shouldn't be void later, make it return the path.
{
    int totalPoints = _width * _height;  // Total number of points in the grid
    std::vector<int> indices(totalPoints);
    std::iota(indices.begin(), indices.end(), 0);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    // Shuffle the indices
    std::shuffle(indices.begin(), indices.end(), std::default_random_engine(seed));

    indices.resize(200);

    StatTracker stats;
    stats.initializeThread();

    int i = 0;
    for (int index : indices) {
        int x = index % _width;
        int y = index / _width;

        setCell(x, y, 2);

        i++;

        if (i > 2) {

            if (!(addAllPaths())) continue; // this initializes the TSP Map and the all the paths

            stats.GlobalTimeStart();

            //_TSPSolve_BruteForce(tspMap);

            _LK_Route2(&_points, &_currentorder, &tspMap);

            //_TSPSolve_heldKarp(tspMap);

            stats.GlobalTimeStop();

            //QString text = myGrid->PrintPath(result.second);
            //WriteConsole(text);
            if (stats.GlobalTimeGet() > 300) {
                stats.StoreGlobalTime(i);
                break;
            }

            stats.StoreGlobalTime(i);


        }
    }

    stats.saveFile2(1);
    stats.quit();

}


void Grid::Solve100_Euclidean() // this shouldn't be void later, make it return the path.
{
    stats.MemUsage();
    stats.StoreGlobalMemory(0);

    for (int i = 1; i < 5; i+=1) {
        resizeGrid(i, i);
        resetGrid();
        fullResetPath();

        std::vector<int> indices(i * i);

        std::iota(indices.begin(), indices.end(), 0);

        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

        std::shuffle(indices.begin(), indices.end(), std::default_random_engine(seed));

        indices.resize(100);

        for (int index : indices) {
            int x = index % i;
            int y = index / i;

            setCell(x, y, 2);
        }

        stats.GlobalTimeStart();

        if (!(addAllPaths())) continue; // this initializes the TSP Map and the all the paths

        stats.GlobalTimeStop();

        stats.StoreGlobalTime(i);
        stats.MemUsage();
        stats.StoreGlobalMemory(i);

    }
    
    stats.saveFile2(1);
    stats.saveFile2(2);
    stats.quit();

}

void Grid::Solve100_Mem() // this shouldn't be void later, make it return the path.
{
    int totalPoints = _width * _height;  // Total number of points in the grid
    std::vector<int> indices(totalPoints);
    std::iota(indices.begin(), indices.end(), 0);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    // Shuffle the indices
    std::shuffle(indices.begin(), indices.end(), std::default_random_engine(seed));

    indices.resize(100);

    stats.initializeThread();

    int i = 0;
    for (int index : indices) {
        int x = index % _width;
        int y = index / _width;

        setCell(x, y, 2);

        i++;

        if (i > 2) {
            if (!(addAllPaths())) continue; // this initializes the TSP Map and the all the paths

            QtWidgetsApplication1::getInstance()->startMainTimer(); // start collecting memory every 10 seconds

            _Genetic_Route(&_points, &_currentorder, &tspMap);

            QtWidgetsApplication1::getInstance()->stopMainTimer(); // start collecting memory every 10 seconds

            stats.StoreGlobalMemory(i);
        }
    }

    stats.saveFile2(2);
    stats.quit();

}