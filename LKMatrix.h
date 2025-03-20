#ifndef LK_MATRIX
#define LK_MATRIX

#include <random>
#include <algorithm>    // std::swap
// We can always use pair<double, double> if QPoint should not be available. i.e. C version
#include <QtCore/QPoint.h>
//using namespace std;
#include <vector>

int random_int(int min, int max);
std::pair<int, int> make_sorted_pair(int x, int y);
std::pair<double, std::vector<int>> greedyRoute(std::vector<std::vector<double>>* e_matrix);
double eu_dist(const int x1, const int y1, const int x2, const int y2);

struct QuadrantNeighbors {
    int Q1 = -1; // NE
    int Q2 = -1; // NW
    int Q3 = -1; // SW
    int Q4 = -1; // SE
};

class LK {
public:
    int size;
    void LKMatrix(std::vector<QPoint>* coords, std::vector<int>* ids, std::vector<std::vector<double>>* E_DistMatrix);
    bool optimizeTour(); // our main optimization algorithm

    std::vector<int>* retrieveTour();
    //void printTour();
    //void printTourIds();

private:

    double swap2(int pos1, int pos2); // 1 or 2-opt switch
    double swap3(int pos1, int pos2); // 3-opt switch

    double getCurrentTourDistance(int type);
    void createQNN();

    double getTempTourPos(int pos);

    // swap edges on a 2 opt switch
    void swap_edges(std::vector<int>& path, int i, int j) {
        std::reverse(path.begin() + i + 1, path.begin() + j + 1);
    }

    std::vector<int> tour; // current tour order
    std::vector<int> temptour; // current tour order
    std::vector<bool> bannedindices; // temp banned indeces
    //int toursize;

    std::vector<QuadrantNeighbors> neighbors; // current tour order
    std::vector<QPoint>* _coords;
    
    //void joinLocations(int i, int j);
    std::vector<std::vector<double>>* distanceMatrix; // euclidean distance matrix very important
};

#endif
