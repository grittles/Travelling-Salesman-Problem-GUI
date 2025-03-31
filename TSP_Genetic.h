#pragma once
#ifndef GENETIC_H
#define GENETIC_H

#include <vector>
#include <algorithm>
#include <QtCore/QPoint.h>
#include "randomnumgen.h"

<<<<<<< HEAD
// THIS GENETIC ALGORITHM IS ENTIRELY PORTED FROM JAVASCRIPT
// THE ORIGINAL AUTHOR IS: Chaoyu Yang
// https://github.com/parano/GeneticAlgorithm-TSP

=======
>>>>>>> 81637397ada2acb512581e3c66c2e3004ee94520

class TSPGenetic {
public:
    int size;
    void Create(std::vector<QPoint>* coords, std::vector<int>* ids, std::vector<std::vector<double>>* E_DistMatrix);
    void optimizeTour(int iterations); // our main optimization algorithm
    std::vector<int>* retrieveTour();
    double getCurrentTourDistance();

private:
    int population_size= 30;
    float CROSSOVER_PROBABILITY = 0.9;
    float MUTATION_PROBABILITY = 0.01;
    int noimprovement = 0;
    bool guaranteed_mutation = false;
    double bestValue = -1;
    int currentGeneration = 0;
    std::vector<std::vector<int>> population;
    std::vector<double> values;
    std::vector<double> fitnessValues;
    std::vector<double> roulette;
    std::vector<int> best;
    std::vector<int> finalbest;
    std::pair<int, double> currentBest;

    std::default_random_engine engine;

    //std::vector<int> tour; // current tour order
    std::vector<QPoint>* _coords;
    std::vector<std::vector<double>>* distanceMatrix; // euclidean distance matrix very important

    void setBestValue();
    std::pair<int, double> getCurrentBest();
    void setRoulette();
    int wheelOut(double rand);
    std::vector<int> randomIndivial(int n);
    double evaluate(std::vector<int> indivial);
    void GAInitialize();
    void GANextGeneration();
    void tribulate();
    void selection();
    void crossover();
    void doCrossover(int x, int y);
    void mutation();
    std::vector<int> doMutate(std::vector<int> seq);
    std::vector<int> pushMutate(std::vector<int> seq);
    std::vector<int> doKick(std::vector<int> seq);
    std::vector<int> getChild(bool useNext, int x, int y);
};

#endif