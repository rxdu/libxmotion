#include <iostream>

#include "reachability/markov_occupancy.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    MarkovOccupancy<10, 5> occupancy(0, 50, 0, 20);

    occupancy.SetupMarkovModel(3, 3 * 3, 2, 1 * 1);

    std::cout << "------------------------------" << std::endl;

    for (int i = 0; i < 5; ++i)
        std::cout << "Occupancy at t_(" << i << ") : \n"
                  << occupancy.GetOccupancyDistribution(i) << "\n" << std::endl;

    return 0;
}