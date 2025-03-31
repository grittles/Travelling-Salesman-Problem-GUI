#include "TSP_Genetic.h"

void TSPGenetic::Create(std::vector<QPoint>* coords, std::vector<int>* ids, std::vector<std::vector<double>>* E_DistMatrix)
{
    _coords = coords; // now we have our coordinates list, this should be ordered the same as our distance matrix
    //tour = *ids;
    size = coords->size(); // set size aka number of nodes
    distanceMatrix = E_DistMatrix;
    population_size = 30;

    //population.resize(population_size, -1);  // Resize and initialize
    values.resize(population_size, -1);
    fitnessValues.resize(population_size, -1);
    roulette.resize(population_size, -1);

    int seed = time(NULL);
    engine.seed(seed);

    srand(seed); // set random seed for random int and float generators

    //std::random_device rd;
    //std::mt19937 g(rd());
}

void TSPGenetic::GAInitialize() {
    for (int i = 0; i < population_size; i++) {
        population.push_back(randomIndivial(size));
    }
    setBestValue();
}

void TSPGenetic::optimizeTour(int iterations)
{
    GAInitialize();

    int i = 0;
    while (i < 1000) {
        GANextGeneration();
        i++;
    }
    finalbest = best;
    finalbest.push_back(finalbest[0]);
}

std::vector<int>* TSPGenetic::retrieveTour()
{
    
    return &finalbest;
}

double TSPGenetic::getCurrentTourDistance() {
    return evaluate(best);
}


void TSPGenetic::GANextGeneration() {
    //currentGeneration++;
    selection();
    crossover();
    mutation();

    setBestValue();
}
void TSPGenetic::tribulate() {
    //for(int i=0; i<size; i++) {
    for (int i = 0 >> 1; i < population_size; i++) {
        population[i] = randomIndivial(size);
    }
}

void TSPGenetic::selection() {
    std::vector<std::vector<int>> parents;
    int initnum = 4;
    parents.push_back(best);
    parents.push_back(population[currentBest.first]);
    parents.push_back(doMutate(best));
    parents.push_back(pushMutate(best));
    

    setRoulette();
    for (int i = initnum; i < population_size; i++) {
        parents.push_back(population[wheelOut(random_float(0,1))]);
    }
    population = parents;
}

void TSPGenetic::crossover() {
    std::vector<int> queue;
    for (int i = 1; i < population_size; i++) {
        if (random_float(0,1) < CROSSOVER_PROBABILITY) {
            queue.push_back(i);
        }
    }

    std::shuffle(queue.begin(), queue.end(), engine);

    for (int i = 0, j = queue.size() - 1; i < j; i += 2) {
        doCrossover(queue[i], queue[i + 1]);
    }
}

void deleteByValue(std::vector<int>* vec, int value) {
    vec->erase(std::remove(vec->begin(), vec->end(), value), vec->end());
}

int getPrevious(std::vector<int>& vec, int index) {
    return vec[(index - 1 + vec.size()) % vec.size()];
}

int getNext(std::vector<int>& vec, int index) {
    return vec[(index + 1) % vec.size()];
}

void TSPGenetic::doCrossover(int x, int y) {
    std::vector<int> child1 = getChild(true, x, y);
    std::vector<int> child2 = getChild(false, x, y);
    population[x] = child1;
    population[y] = child2;
}

std::vector<int> TSPGenetic::getChild(bool useNext, int x, int y) {
    std::vector<int> solution;
    std::vector<int> px = population[x]; // Assuming deep copy is handled by vector
    std::vector<int> py = population[y];

    int c = px[random_int(0,px.size()-1)]; // Implement randomNumber()
    solution.push_back(c);

    while (px.size() > 1) {
        int index_c = std::find(px.begin(), px.end(), c) - px.begin();
        int dx = useNext ? getNext(px, index_c) : getPrevious(px, index_c);
        int dy = useNext ? getNext(py, std::find(py.begin(), py.end(), c) - py.begin()) :
            getPrevious(py, std::find(py.begin(), py.end(), c) - py.begin());

        deleteByValue(&px, c);
        deleteByValue(&py, c);
        c = (*distanceMatrix)[c][dx] < (*distanceMatrix)[c][dy] ? dx : dy;
        solution.push_back(c);
    }
    return solution;
}
void TSPGenetic::mutation() {
    for (int i = 1; i < population_size; i++) {
        if (random_float(0,1) < MUTATION_PROBABILITY) {
            if (random_float(0,1) > 0.5) {
                population[i] = pushMutate(population[i]);
            }
            else {
                population[i] = doMutate(population[i]);
            }
            i--;
        }

        if (guaranteed_mutation == true) {
            population[i] = doKick(population[i]);
            guaranteed_mutation = false;
            noimprovement = 0;
        }
    }
}

// 2-opt swap?
std::vector<int> TSPGenetic::doMutate(std::vector<int> seq) {
    // m and n refers to the actual index in the array
    // m range from 0 to length-2, n range from 2...length-m
    int m = random_int(0, seq.size() - 2);
    int n = random_int(m + 1, seq.size());

    std::reverse(seq.begin() + m, seq.begin() + n);

    return seq;
}

std::vector<int> TSPGenetic::doKick(std::vector<int> seq) {
    // m and n refers to the actual index in the array
    // m range from 0 to length-2, n range from 2...length-m
    int s = seq.size() - 1;
    int m = random_int(0, s);
    int n = random_int(0, s);

    while (m == n) {
        n = random_int(0, s);
    }

    std::swap(seq[m], seq[n]);

    return seq;
}

std::vector<int> TSPGenetic::pushMutate(std::vector<int> seq) {
    int m = random_int(0, seq.size() >> 1);
    int n = random_int(m + 2, seq.size());
    
    std::vector<int> s1(seq.begin(), seq.begin() + m);
    std::vector<int> s2(seq.begin() + m, seq.begin() + n);
    std::vector<int> s3(seq.begin() + n, seq.end());

    std::vector<int> result;

    // Reserve enough space for all elements to avoid reallocation
    result.reserve(s2.size() + s1.size() + s3.size());

    // Concatenate s2, s1, and s3 in that order
    result.insert(result.end(), s2.begin(), s2.end());
    result.insert(result.end(), s1.begin(), s1.end());
    result.insert(result.end(), s3.begin(), s3.end());

    return result;
}

void TSPGenetic::setBestValue() {
    for (int i = 0; i < population_size; i++) {
        values[i] = evaluate(population[i]);
    }
    currentBest = getCurrentBest();
    double cbest = currentBest.second;

    if (bestValue == -1 || bestValue > cbest) {
        best = population[currentBest.first];
        bestValue = cbest;
        noimprovement = 0;
    }
    else {
        noimprovement++;
    }


    if (noimprovement > 3) {
        guaranteed_mutation = true;
    }
}
std::pair<int, double> TSPGenetic::getCurrentBest() {
    double bestP = 0,
        currentBestValue = values[0];

    for (int i = 1; i < population_size; i++) {
        if (values[i] < currentBestValue) {
            currentBestValue = values[i];
            bestP = i;
        }
    }
    return std::make_pair(bestP, currentBestValue);
}

void TSPGenetic::setRoulette() {
    //calculate all the fitness
    for (int i = 0; i < values.size(); i++) { fitnessValues[i] = 1.0 / values[i]; }
    //set the roulette
    int sum = 0;
    for (int i = 0; i < fitnessValues.size(); i++) { sum += fitnessValues[i]; }
    for (int i = 0; i < roulette.size(); i++) { roulette[i] = fitnessValues[i] / sum; }
    for (int i = 1; i < roulette.size(); i++) { roulette[i] += roulette[i - 1]; }
}

int TSPGenetic::wheelOut(double rand) {
    int i;
    for (i = 0; i < roulette.size(); i++) {
        if (rand <= roulette[i]) {
            return i;
        }
    }
}

std::vector<int> TSPGenetic::randomIndivial(int n) {
    std::vector<int>  a;
    for (int i = 0; i < n; i++) {
        a.push_back(i);
    }
    std::shuffle(a.begin(), a.end(), engine);
    return a;
}

double TSPGenetic::evaluate(std::vector<int> indivial) {
    int size = indivial.size();
    double sum = (*distanceMatrix)[indivial[0]][indivial[size-1]];
    for (int i = 0; i < size-1; i++) {
        sum += (*distanceMatrix)[indivial[i]][indivial[i + 1]];
    }
    return sum;
}
