#include <iostream>

#include "reachability/markov_occupancy.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    MarkovOccupancy<10, 5> occupancy(0, 50, 0, 20);

    occupancy.SetupMarkovModel(33, 3 * 3, 5, 1 * 1);

    std::cout << "------------------------------" << std::endl;

    for (int i = 0; i < 12; i = i + 3)
        std::cout << "Occupancy at t_(" << i << ") : \n"
                  << occupancy.GetOccupancyDistribution(i) << "\n" << std::endl;

    return 0;
}