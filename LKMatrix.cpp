#include "LKMatrix.h"
//#include "Quadtree.h"
#include <cmath>
#include <set>
#include <iostream>
#include <cassert>
#include <cstdlib>
#include <ctime>
//#include <sys/time.h>
//#include <stdio.h>
//#include <unistd.h>



// Add matrix and coordinates to the LK object and initialize stuff
void LK::LKMatrix(std::vector<QPoint>* coords, std::vector<int>* ids, std::vector<std::vector<double>>* E_DistMatrix) {
    _coords = coords; // now we have our coordinates list, this should be ordered the same as our distance matrix
    tour = *ids;
    temptour = *ids;
    size = coords->size(); // set size aka number of nodes

    bannedindices.resize(size, 0);  // Resize and initialize bannedindices to keep track of which indices we've banned

    distanceMatrix = E_DistMatrix;

    srand(time(NULL)); // set random seed

    //QuadtreeNode* root = new QuadtreeNode(0, 0, 50, 50);  // Define a root node covering a certain area

}

// this is for 2-opt switch
// pos1 and pos2 is in relation to the current tour, euclidean distance matrix is good
// remember DO NOT include i = 0.
double LK::swap2(int i, int j) {
    // need E_Distance matrix
    // need current tour
    // need temp tour
    // returns gain

    if (i > j) std::swap(i, j);

    double oldDist = 0, newDist = 0;

    oldDist = (*distanceMatrix)[temptour[i]][temptour[i+1]] + // i-1 -> i
              (*distanceMatrix)[temptour[j]][temptour[j+1]];  // j   -> j+1

    newDist = (*distanceMatrix)[temptour[i]][temptour[j]] + // i-1 -> j
              (*distanceMatrix)[temptour[i+1]][temptour[j+1]];  // i   -> j+1

    if (oldDist > newDist) {
            std::reverse(temptour.begin() + i, temptour.begin() + (j + 1));
            return oldDist - newDist;  // Return the gain from the swap
    } // newdist should be smaller, for a positive gain.

    return 0;
}

// for 3 swaps, rules are different
double LK::swap3(int i, int j) {
    // need E_Distance matrix
    // need current tour
    // need temp tour
    // returns gain

    if (j==i) // i and j can't be equal or adjacent
        return 0;

    if (i > j) std::swap(i, j);
    if (!((j - i) >= 1)) // i and j can't be equal or adjacent
        j+=1;

    double oldDist = 0, newDist = 0;

    oldDist = (*distanceMatrix)[temptour[i-1]][temptour[i]] + // i-1 -> i
              (*distanceMatrix)[temptour[i]][temptour[i+1]] + // i   -> i+1

              (*distanceMatrix)[temptour[j]][temptour[j+1]]; // j  -> j+1

    newDist = (*distanceMatrix)[temptour[i-1]][temptour[i+1]] + // i-1 -> i+1

              (*distanceMatrix)[temptour[j]][temptour[i]] + // j -> i
              (*distanceMatrix)[temptour[i]][temptour[j+1]]; // i -> j+1

    if (oldDist > newDist) {
        int elementToMove = temptour[i]; // store temp variable because we are deleting it.
        temptour.erase(temptour.begin() + i); // delete element at i
        // insert element at i between old j and j+1. Since vector size has decreased by 1,
        // element at old j is at j-1 so we can insert it at pos j
        temptour.insert(temptour.begin() + j, elementToMove); 

        return oldDist - newDist;  // Return the gain from the swap
    } // newdist should be smaller, for a positive gain.

    return 0;
}

void LK::createQNN() {

    neighbors.resize(size);
    neighborsCombined.resize(size);

    for (int i = 0; i < size; i++) {
        QPoint* temp = &(*_coords)[i];

        int x1 = temp->x();
        int y1 = temp->y();

        for (int j = 0; j < size; j++) {

            // skip if i and j are the same ior
            if (i == j) {
                continue;
            }

            // this could have been done in pre-processing, but w/e
            QPoint* temp2 = &(*_coords)[j];

            int x2 = temp2->x();
            int y2 = temp2->y();

            // Determine the quadrant using conditional checks
            bool isEast = x2 >= x1;
            bool isNorth = y2 <= y1;

            std::vector<int>* the_Q = nullptr; // the code below selects which quadrant to focus on by pointer

            if (isEast && isNorth) {// NE = Q1
                the_Q = &(neighbors[i]).Q1;
            }
            else if (!isEast && isNorth) { // NW = Q2
                the_Q = &(neighbors[i]).Q2;
            }
            else if (!isEast && !isNorth) { // SW = Q3
                the_Q = &(neighbors[i]).Q3;
            }
            else {
                the_Q = &(neighbors[i]).Q4; // SE = Q4
            }

            int tsize = the_Q->size();
            if (tsize < QNNMaxNeighbors) { // if theQindex is -1 there is no neighbor in this quadrant so choose this one
                the_Q->push_back(j);
                continue;
            }

            // create a loop for any tsize if we want more quadrant neighbors later on
            // loops until it finds an index it's shorter than.
            double newdist = (*distanceMatrix)[i][j];
            for (int k = 0; k < tsize; k++) {
                // grab distancematrix index
                int theQindex = (*the_Q)[k];

                // grab comparison distance
                double olddist = (*distanceMatrix)[i][theQindex];
                if (newdist > olddist) { // if our comparison distance is older then go to the next
                    continue;
                }
                else {
                    (*the_Q)[k] = j; // replace the value. This is in relation to the euclidean distance matrix and NOT the tour
                    break;
                }
            }
        }
        
        // after getting our QNN's combine them into a pool for the node
        QuadrantNeighbors* n = &neighbors[i];

        std::vector<int>* n2 = &neighborsCombined[i];

        n2->reserve(n->Q1.size() + n->Q2.size() + n->Q3.size() + n->Q4.size()); // Optimize allocations

        n2->insert(n2->end(), n->Q1.begin(), n->Q1.end());
        n2->insert(n2->end(), n->Q2.begin(), n->Q2.end());
        n2->insert(n2->end(), n->Q3.begin(), n->Q3.end());
        n2->insert(n2->end(), n->Q4.begin(), n->Q4.end());
    }
}

// todo: Implement QNN and random kicks
bool LK::optimizeTour() {
    // this could be improved vastly with multithreading if we knew how to do it
    if (distanceMatrix->size() != size) {
        return false; // this is a size mismatch which means we're screwed
    }

    int diff;
    float old_distance = 0;
    float new_distance = 0;

    int x_coord_sum = 0;
    int y_coord_sum = 0;

    // step 1: Create Initial Tour using Greedy Algorithm
    auto result = greedyRoute(distanceMatrix);
    old_distance = result.first; // todo: verify that this gives an accurate distance
    tour = result.second; // this will be our current tour
    new_distance = getCurrentTourDistance(0); // verify this too

    temptour = result.second; // this will be our current tour

    bool foundImprovement = true;
    bool tempImprovement = true;
    while (foundImprovement) {
        //tour = temptour;
        foundImprovement = false;
        for (int i = 0; i < size - 1; i++) {
            tempImprovement = false;
            int* index = &(bannedindices[i]);

            if ((*index) > 5) continue;

            for (int j = i + 2; j < size; j++) {
                int k = (j + 1) % size;  // Circular reference
                float lengthDelta =
                    -(*distanceMatrix)[temptour[i]][temptour[i + 1]] - (*distanceMatrix)[temptour[j]][temptour[k]]
                    + (*distanceMatrix)[temptour[i]][temptour[j]] + (*distanceMatrix)[temptour[i + 1]][temptour[k]];


                if (lengthDelta < -0.0001) {
                    swap_edges(temptour, i, j);
                    foundImprovement = true;
                    tempImprovement = true;
                }
            }

            if (!tempImprovement) (*index)++;

        }

    }

    tour = temptour;

    return true;

}

// todo: Implement banned indeces
// also: the current tour system is really stupid, it'd be easier
// to remove the wraparound at the end then we could do swaps with pos 0
// after 3 consecutive tours with no improvements for a particular node, ban the index.
bool LK::optimizeTour2() {
    // this could be improved vastly with multithreading if we knew how to do it
    if (distanceMatrix->size() != size) {
        return false; // this is a size mismatch which means we're screwed
    }

    int diff;
    float old_distance = 0;
    float new_distance = 0;

    int x_coord_sum = 0;
    int y_coord_sum = 0;

    // step 1: Create Initial Tour using Greedy Algorithm
    auto result = greedyRoute(distanceMatrix);
    old_distance = result.first; // todo: verify that this gives an accurate distance
    tour = result.second; // this will be our current tour
    //new_distance = getCurrentTourDistance(0); // verify this too
    new_distance = old_distance;


    temptour = result.second; // this will be our current tour

    createQNN();
    bool foundImprovement = false;
    bool tempImprovement = true;
    int kicks = (10 > size) ? size : 10;
    while (kicks >= 0) {

        foundImprovement = false;

        for (int i = 0; i < size - 1; i++) { // for each node

            tempImprovement = false;
            int* index = &(bannedindices[i]);

            if ((*index) > 3) continue;

            std::vector<int> n = neighborsCombined[i]; // grab neighbors
            int nsize = n.size();
            for (int p = 0; p < nsize; p++) { // for each neighbor
                int i_ = getTempTourPos(i);
                int j = getTempTourPos(n[p]); // should come up with a better way to do this. very inefficient here

                if (i_ > j) {
                    std::swap(i_, j);
                }

                int diff = j - i_;

                int k = (j + 1);  // Circular reference

                double dist2 = (*distanceMatrix)[temptour[j]][temptour[k]];

                double lengthDelta;

                if (diff == 1) {

                    if (i_ > 0) {
                        double dist8 = (*distanceMatrix)[temptour[i_ - 1]][temptour[i_]];
                        // double dist2
                        double dist9 = (*distanceMatrix)[temptour[i_ - 1]][temptour[j]];
                        double dist10 = (*distanceMatrix)[temptour[i_]][temptour[k]];
                        lengthDelta =
                            dist8 + dist2
                            - dist9 - dist10;

                        if (lengthDelta > 0.001) {
                            std::swap(temptour[i_], temptour[j]);
                            foundImprovement = true;
                            tempImprovement = true;
                            continue;
                        }
                    }
                    continue;
                }

                double dist1 = (*distanceMatrix)[temptour[i_]][temptour[i_ + 1]];
                // double dist2 = (*distanceMatrix)[temptour[j]][temptour[k]]; // dist2 already calculated
                double dist3 = (*distanceMatrix)[temptour[i_]][temptour[j]];
                double dist4 = (*distanceMatrix)[temptour[i_ + 1]][temptour[k]];

                lengthDelta =
                    dist1 + dist2
                    - dist3 - dist4;

                if (lengthDelta > 0.001) {
                    swap_edges(temptour, i_, j);
                    foundImprovement = true;
                    tempImprovement = true;
                }
                else {

                    // attempt 3-opt switch
                    if (diff < 3 || i_ == 0) continue;

                    double dist5 = (*distanceMatrix)[temptour[j - 1]][temptour[j]];
                    double dist6 = (*distanceMatrix)[temptour[j]][temptour[i_ + 1]];
                    double dist7 = (*distanceMatrix)[temptour[j - 1]][temptour[k]];

                    lengthDelta =
                        dist1 + dist2 + dist5
                        - dist3 - dist6 - dist7;

                    if (lengthDelta > 0.001) {
                        swap_edges3(temptour, i_, j);
                        foundImprovement = true;
                        tempImprovement = true;

                    }
                    else {
                        //double dist8 = (*distanceMatrix)[temptour[i]][temptour[i_ + 1]];
                        //double dist9 = (*distanceMatrix)[temptour[j - 1]][temptour[k]];
                    }

                }

            }
            if (!tempImprovement) (*index)++;
        }

        if (foundImprovement == true) {

            //old_distance = new_distance;
            new_distance = getCurrentTourDistance(1);

            if (old_distance > new_distance) {
                tour = temptour;
                old_distance = new_distance;
                continue;
            }
        }

        // if no improvement was found...

        kicks--;

        if (kicks == -1) return true;

        // replace temptour with current best tour;
        temptour = tour;

        if (size < 5) continue;

        // do random kick
        int int1 = random_int(1, size - 1); // size -1 because we can't be swapping the start and finish
        int int2 = random_int(1, size - 1);

        while (int1 == int2) {
            int2 = random_int(1, size - 1);
        }

        // perform random swap.
        std::swap(temptour[int1], temptour[int2]);
    }

    return true;

}

std::vector<int>* LK::retrieveTour()
{
    return &tour;
}

// finds the position of a node in the temptour
double LK::getTempTourPos(int node) {
    auto it = find(temptour.begin(), temptour.end(), node);

    if (it == temptour.end()) {
        return -1;
    }
        // get the index
    return std::distance(temptour.begin(), it); // get the current index of our target in the tour
}

double LK::getCurrentTourDistance(int type=0) {
    double distance = 0;
  
    for (int i = 0; i < size; i++) { // yes im lazy sorry guys
        if (type == 0) {
            distance += (*distanceMatrix)[tour[i]][tour[i + 1]]; // assuming we do include the start node in our path
        }
        else {
            distance += (*distanceMatrix)[temptour[i]][temptour[i + 1]]; // assuming we do include the start node in our path
        }
    }
    return distance;
}

// implement greedy algorithm to feed into LK Algorithm later
// looks like a lot but this is actually a pretty simple algorithm
// todo: add this algorithm to a separate file and a separate header
std::pair<double, std::vector<int>> greedyRoute(std::vector<std::vector<double>>* e_matrix) {

    // init vars
    int num = e_matrix->size(); // it's n*n so no need to grab another size
    std::vector<int> tour; // Stores the order of visited cities
    std::vector<bool> visited(num, false); // Track visited cities

    float totalDistance = 0.0;
    int currentNode = 0;
    visited[currentNode] = true;
    tour.push_back(currentNode);

    // Main loop to visit each city exactly once
    // i is not used since we're tracking using currentNode
    for (int i = 1; i < num; ++i) {
        // Find the nearest unvisited city
        float minDist = -1;
        int nextCity = -1;
        for (int j = 0; j < num; ++j) {
            if (!visited[j] && ((minDist == -1) || ((*e_matrix)[currentNode][j] < minDist))) {
                minDist = (*e_matrix)[currentNode][j];
                nextCity = j;
            }
        }

        // Visit the next city
        visited[nextCity] = true;
        tour.push_back(nextCity);
        totalDistance += minDist;
        currentNode = nextCity;
    }

    // Return to the starting city
    totalDistance += (*e_matrix)[tour.back()][tour[0]];
    tour.push_back(tour[0]);

    return { totalDistance , tour };

}