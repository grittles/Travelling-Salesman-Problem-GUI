// Grid.cpp
#include "randomnumgen.h"




float random_float(float min, float max) {
    return ((float)rand() / RAND_MAX) * (max - min) + min;
};

int random_int(int min, int max) {
    return min + rand() % (max - min + 1);
};