#include <iostream>

#include "reachability/markov_occupancy.hpp"

#include "stopwatch.hpp"

using namespace xmotion;

int main()
{
    MarkovOccupancy<10, 5> occupancy(0, 50, 0, 20);

    occupancy.SetupMarkovModel(3, 3 * 3, 5, 1 * 1);

    const int32_t max_i = 12;
    occupancy.Propagate(max_i);

    std::cout << "------------------------------" << std::endl;

    for (int32_t i = 0; i < max_i; i = i + 3)
    {
        std::cout << "Occupancy at t_(" << i << ") : \n"
                  << occupancy.GetOccupancyDistribution(i) << "\n" << std::endl;
        // std::cout << "Interval occupancy at t_(" << i << ") : \n"
        //           << occupancy.GetIntervalOccupancyDistribution(i) << "\n" << std::endl;
    }

    return 0;
}