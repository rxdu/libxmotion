#include <iostream>

#include "reachability/markov_occupancy.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    MarkovOccupancy<10,10> occupancy(0, 50, 0, 20);

    occupancy.SetupMarkovModel();

    return 0;
}