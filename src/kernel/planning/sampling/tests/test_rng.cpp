#include <iostream>
#include <cstdint>
#include <iomanip>
#include <map>

#include "sampling/details/sampler/rand_num_gen.hpp"

using namespace librav;

int main(int argc, char *argv[])
{
    RandNumGen gen;

    int mean = gen.UniformInt(1, 6);
    std::cout << "Randomly-chosen mean: " << mean << '\n';

    std::map<int, int> hist;
    for (int n = 0; n < 10000; ++n)
    {
        ++hist[std::round(gen.Gaussian(mean, 2))];
    }
    std::cout << "Normal distribution around " << mean << ":\n";
    for (auto p : hist)
    {
        std::cout << std::fixed << std::setprecision(1) << std::setw(2)
                  << p.first << ' ' << std::string(p.second / 200, '*') << '\n';
    }

    return 0;
}