#include "LKMatrix.h"
#include "Quadtree.h"
#include <cmath>
#include <set>
#include <iostream>
#include <cassert>
#include <cstdlib>
#include <ctime>
//#include <sys/time.h>
//#include <stdio.h>

//#include <unistd.h>

// wish I knew about this earlier lmfao
//using namespace std;

int random_int(int min, int max) {
    return min + rand() % (max - min + 1);
}

double eu_dist(const int x1, const int y1, const int x2, const int y2) {

    double distance_x = std::abs(x1 - x2);
    double distance_y = std::abs(y1 - y2);
    // Return Euclidean distance
    return sqrt(distance_x * distance_x + distance_y * distance_y);
}


// Add matrix and coordinates to the LK object and initialize stuff
void LK::LKMatrix(std::vector<QPoint>* coords, std::vector<int>* ids, std::vector<std::vector<double>>* E_DistMatrix) {
  _coords = coords; // now we have our coordinates list, this should be ordered the same as our distance matrix
  tour = *ids;
  temptour = *ids;
  size = coords->size(); // set size aka number of nodes

  bannedindices.resize(size+1, false);  // Resize and initialize bannedindices to keep track of which indices we've banned

  distanceMatrix = E_DistMatrix;

  srand(time(NULL)); // set random seed

  QuadtreeNode* root = new QuadtreeNode(0, 0, 50, 50);  // Define a root node covering a certain area

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


//void LK::createQNN() {
//
//    neighbors.resize(size);
//
//    for (int i = 1; i < size; i++) {
//        QPoint* temp = &(*_coords)[i];
//
//        int x1 = temp->x();
//        int y1 = temp->y();
//
//        for (int j = 1; j < size; j++) {
//
//            // skip if i and j are the same ior
//            if (i == j) {
//                continue;
//            }
//
//            QPoint* temp2 = &(*_coords)[j];
//
//            int x2 = temp2->x();
//            int y2 = temp2->y();
//
//            // Determine the quadrant using conditional checks
//            bool isEast = x2 >= x1;
//            bool isNorth = y2 <= y1;
//
//            int* the_Q = nullptr; // the code below selects which quadrant to focus on by pointer
//
//            if (isEast && isNorth) {
//                the_Q = &(neighbors[i]).Q1;
//            }
//            else if (!isEast && isNorth) {
//                the_Q = &(neighbors[i]).Q2;
//            }
//            else if (!isEast && !isNorth) {
//                the_Q = &(neighbors[i]).Q3;
//            }
//            else {
//                the_Q = &(neighbors[i]).Q4;
//            }
//            int theQindex = *the_Q;
//            if (theQindex == -1) { // if theQindex is -1 there is no neighbor in this quadrant so choose this one
//                *the_Q = j;
//                continue;
//            }
//
//            double newdist = eu_dist(x1, y1, x2, y2);
//
//            temp2 = &(*_coords)[theQindex];
//            double olddist = eu_dist(x1, y1, temp2->x(), temp2->y());
//
//            if (olddist > newdist) {
//                *the_Q = j; // replace the value. This is in relation to the euclidean distance matrix and NOT the tour
//                continue;
//            }
//
//        }
//    }
//}

void LK::createQNN() {

    neighbors.resize(size);

    for (int i = 1; i < size; i++) {
        QPoint* temp = &(*_coords)[i];

        int x1 = temp->x();
        int y1 = temp->y();

        for (int j = 1; j < size; j++) {

            // skip if i and j are the same ior
            if (i == j) {
                continue;
            }

            QPoint* temp2 = &(*_coords)[j];

            int x2 = temp2->x();
            int y2 = temp2->y();

            // Determine the quadrant using conditional checks
            bool isEast = x2 >= x1;
            bool isNorth = y2 <= y1;

            int* the_Q = nullptr; // the code below selects which quadrant to focus on by pointer

            if (isEast && isNorth) {
                the_Q = &(neighbors[i]).Q1;
            }
            else if (!isEast && isNorth) {
                the_Q = &(neighbors[i]).Q2;
            }
            else if (!isEast && !isNorth) {
                the_Q = &(neighbors[i]).Q3;
            }
            else {
                the_Q = &(neighbors[i]).Q4;
            }
            int theQindex = *the_Q;
            if (theQindex == -1) { // if theQindex is -1 there is no neighbor in this quadrant so choose this one
                *the_Q = j;
                continue;
            }

            double newdist = eu_dist(x1, y1, x2, y2);

            temp2 = &(*_coords)[theQindex];
            double olddist = eu_dist(x1, y1, temp2->x(), temp2->y());

            if (olddist > newdist) {
                *the_Q = j; // replace the value. This is in relation to the euclidean distance matrix and NOT the tour
                continue;
            }

        }
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
    while (foundImprovement) {
        //tour = temptour;
        foundImprovement = false;
        for (int i = 0; i < size - 1; i++) {
            for (int j = i + 2; j < size; j++) {
                int k = (j + 1) % size;  // Circular reference
                float lengthDelta =
                    -(*distanceMatrix)[temptour[i]][temptour[i + 1]] - (*distanceMatrix)[temptour[j]][temptour[k]]
                    + (*distanceMatrix)[temptour[i]][temptour[j]] + (*distanceMatrix)[temptour[i + 1]][temptour[k]];


                if (lengthDelta < 0) {
                    swap_edges(temptour, i, j);
                    foundImprovement = true;
                }
            }
        }

    }

    tour = temptour;

    return true;

}


// this code is broken so rather than fix it we'll adjust the method above since that one works well
//bool LK::optimizeTour() {
//    // this could be improved vastly with multithreading if we knew how to do it
//    if (distanceMatrix->size() != size) {
//        return false; // this is a size mismatch which means we're screwed
//    }
//
//    int diff;
//    float old_distance = 0;
//    float new_distance = 0;
//
//    int x_coord_sum = 0;
//    int y_coord_sum = 0;
//
//    // step 1: Create Initial Tour using Greedy Algorithm
//    auto result = greedyRoute(distanceMatrix);
//    old_distance = result.first; // todo: verify that this gives an accurate distance
//    tour = result.second; // this will be our current tour
//    new_distance = getCurrentTourDistance(0); // verify this too
//
//    temptour = result.second; // this will be our current tour
//
//    // first build our quadrant nearest neighbor list using our coords list.
//    // TODO: REPLACE THIS NESTED LOOP WITH A QUAD-TREE. PERHAPS A CLASS OBJECT
//    createQNN();
//
//    int kicks = 10;
//    // current tour = tour, which is an int vector, representing indices 
//    for (int z = 0; z < kicks; z++) {
//
//        int nogain = 0;
//        int index = 1; // skip index  0
//        while (nogain < 3) {
//            double tempgain = 0;
//            // check max size, exclude final index as that's the loop point
//            // check if current index is banned.
//            if (index == size - 1) {
//                index = 1; // start over if we reached the end.
//            }
//
//            if (bannedindices[index] == true) {
//                index++;
//                continue;
//            };
//
//            // we're iterating through the tour itself. So if it's 2->7->9... index=1 would be tempindex = 7
//            // which would be the 7th node in the euclidean distance matrix
//            int tempindex = temptour[index];
//
//            // now we grab the closest neighbors and check for any swaps
//            QuadrantNeighbors n = neighbors[tempindex];
//
//            // combine them together so we can just loop through them. Lazy fix.
//            vector<int> quadrants = { n.Q1, n.Q2, n.Q3, n.Q4 };
//
//            for (int loc : quadrants) { // loc = location of index in quadrant
//
//                if (loc == -1) { // loc = 1 means there's no node in that quadrant
//                    continue;
//                }
//
//                int index2 = getTempTourPos(loc);
//
//                if (index2 == size) {
//                            bannedindices[index2 + 1] = true;
//                        }
//
//                if (index2 != -1) {
//                    if (bannedindices[index2] == true) {
//                        continue; // skip to next iteration? use a breakpoint to double check that's what this code is actually doing
//                    }
//
//                    double gain;
//                    gain = swap2(index, index2); // tour indices swap
//
//                    if (gain > 0) { // if no gain, go to the next QNN, there's no waste here.
//                        
//                        // this is for debug, let's verify our gains
//                        double tempdistance = getCurrentTourDistance(1);
//                        double tempdistance2 = getCurrentTourDistance(0);
//
//                        bannedindices[index] = true;
//                        bannedindices[index2] = true;
//
//                        if (index2 == size) {
//                            bannedindices[index2 + 1] = true;
//                        }
//
//                        bannedindices[index + 1] = true;
//                        //bannedindices[index2 + 1] = true;
//
//                        tempgain += gain;
//                        break; // break the for loop. Again, check with a breakpoint here for expected behavior
//                    }
//                    
//                    // if no gain from 2-opt, we try a 3-opt swap rather than doing a whole other set of swaps later.
//                    else {
//                        
//                            
//                        // need at least 5 nodes available to perform 3-opt
//                        if (!((size) > 4)) {
//                            continue;
//                        }
//
//                        // 
//                        if (!((index > 1) || (size - index2) > 1)) { // cannot select end nodes
//                            continue;
//                        }
//                        gain = swap3(index, index2); // tour indices swap
//
//                        if (gain > 0) { 
//                                
//                            // with a 3-opt switch we actually only effectively swap one node to index2 position
//                            bannedindices[index2] = true;
//
//                            tempgain += gain;
//                            break; // break the for loop. Again, check with a breakpoint here for expected behavior
//                        }
//                    }
//                }
//            }
//
//            index += 1;
//
//            if (!(tempgain > 0)) {
//                nogain += 1;
//            }
//            else {
//                double tempdistance = getCurrentTourDistance(1);
//                if (old_distance > tempdistance) {
//                    tour = temptour;
//                    old_distance = tempdistance;
//                }
//                //old_distance = getCurrentTourDistance(); // guess we don't really need the new_distance var
//            }
//
//
//            // reset our bannedindices to false so we can try again.
//            tempgain = 0;
//            std::fill(bannedindices.begin(), bannedindices.end(), false);
//
//            }
//
//        // we get to this point if we've had 3 tours with no improvements, so, do a random kick to the temptour and start again
//        int int1 = random_int(1, size - 1); // size -1 because we can't be swapping the start and finish
//        int int2 = random_int(1, size - 1);
//
//        while (int1 == int2) {
//            int2 = random_int(1, size - 1);
//        }
//
//        temptour = tour;
//        // perform random swap.
//        swap(temptour[int1], temptour[int2]);
//
//        // remember, at this point, we've saved our best tour so far to tour. So, we're not ruining anything by doing this.
//        // if no improvement is found from this swap, it'll keep the next best one.
//        // after 10 kicks, we're done.
//    }
//
//    return true;
//
//}

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