#pragma once
#ifndef LK_MATRIX
#define LK_MATRIX

//#include <random>
#include <algorithm>    // std::swap
// We can always use pair<double, double> if QPoint should not be available. i.e. C version
#include <QtCore/QPoint.h>
//using namespace std;
#include <vector>
#include "randomnumgen.h"


std::pair<double, std::vector<int>> greedyRoute(std::vector<std::vector<double>>* e_matrix);

struct QuadrantNeighbors {
    std::vector<int> Q1 = {}; // NE
    std::vector<int> Q2 = {}; // NW
    std::vector<int> Q3 = {}; // SW
    std::vector<int> Q4 = {}; // SE
};

class LK {
public:
    int size;
    void LKMatrix(std::vector<QPoint>* coords, std::vector<int>* ids, std::vector<std::vector<double>>* E_DistMatrix);
    bool optimizeTour(); // our main optimization algorithm
    bool optimizeTour2(); // our main optimization algorithm (testing phase)

    std::vector<int>* retrieveTour();
    //void printTour();
    //void printTourIds();
    double getCurrentTourDistance(int type);

private:

    double swap2(int pos1, int pos2); // 1 or 2-opt switch
    double swap3(int pos1, int pos2); // 3-opt switch

    void createQNN();

    double getTempTourPos(int pos);

    // swap edges on a 2 opt switch
    void swap_edges(std::vector<int>& path, int i, int j) {
        std::reverse(path.begin() + i + 1, path.begin() + j + 1);
    }

    // swap edges on a very specific type of 3 opt switch
    // this is NOT the general 3-opt switch.
    void swap_edges3(std::vector<int>& path, int i, int j) {
        
        int finalpos = (j > i) ? i : (i - 1);

        int elementToMove = path[j]; // store temp variable because we are deleting it.
        path.erase(path.begin() + j); // delete element at i
        
        path.insert(path.begin() + finalpos, elementToMove);
    }

    std::vector<int> tour; // current tour order
    std::vector<int> temptour; // current tour order
    std::vector<int> bannedindices; // temp banned indeces
    //int toursize;

    std::vector<QuadrantNeighbors> neighbors; // current tour order
    std::vector<std::vector<int>> neighborsCombined; // current tour order
    std::vector<QPoint>* _coords;
    
    //void joinLocations(int i, int j);
    std::vector<std::vector<double>>* distanceMatrix; // euclidean distance matrix very important

    // max neighbors per quadrant per node. 1->4, 2->8.
    int QNNMaxNeighbors = 2;
};

#endif
