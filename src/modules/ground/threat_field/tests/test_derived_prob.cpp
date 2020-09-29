#include <cmath>
#include <vector>
#include <utility>
#include <iostream>

#include "random/bigaussian_sampler.hpp"

#include "stopwatch/stopwatch.h"
#include "logging/csv_logger.hpp"

using namespace librav;

struct TransformFunc
{

    double operator()(double p1, double p2, double v1, double v2)
    {
        double beta = std::atan2(v2, v1);

        double omega11 = std::cos(beta) * std::cos(beta) / (2 * sigma_f * sigma_f) +
                         std::sin(beta) * std::sin(beta) / (2 * sigma_s * sigma_s);
        double omega12 = -std::cos(2 * beta) / (4 * sigma_f * sigma_f) +
                         std::sin(2 * beta) / (4 * sigma_s * sigma_s);
        double omega21 = omega12;
        double omega22 = std::sin(beta) * std::sin(beta) / (2 * sigma_f * sigma_f) +
                         std::cos(beta) * std::cos(beta) / (2 * sigma_s * sigma_s);

        double numerator = std::exp(-(p1 * p1 * omega11 + p1 * p2 * omega21 +
                                      p1 * p2 * omega12 + p2 * p2 * omega22));
        double denominator = 1 + std::exp(-alpha * (v1 * p1 + v2 * p2));

        return numerator / denominator;
    }

    static constexpr double alpha = 0.05;
    static constexpr double sigma_f = (4.8 / 2) * (4.8 / 2);
    static constexpr double sigma_s = (1.8 * 4 / 5) * (1.8 * 4 / 5);
};

int main()
{
    stopwatch::StopWatch timer;
    CsvLogger logger("monte_carlo_sim", "/home/rdu");

    TransformFunc trans;
    // BiGaussianSampler pos_sampler(10, 10, 0);
    // BiGaussianSampler pos_sampler(3, 3, 0);
    // BiGaussianSampler vel_sampler(2, 1, 0);
    BiGaussianSampler pos_sampler(2, 2, 0);
    BiGaussianSampler vel_sampler(1.0, 0.5, 0);

    const int32_t sim_num = 10000;
    std::vector<std::pair<double, double>> pos_samples;
    std::vector<std::pair<double, double>> vel_samples;
    std::vector<double> results;
    double posx, posy, velx, vely;

    timer.tic();
    for (int32_t i = 0; i < sim_num; ++i)
    {
        pos_sampler.Sample(&posx, &posy);
        vel_sampler.Sample(&velx, &vely);
        pos_samples.emplace_back(std::make_pair(posx, posy));
        vel_samples.emplace_back(std::make_pair(velx, vely));

        // double y = trans(5 + posx, 1 + posy, 10 + velx, 2 + vely);
        // double y = trans(posx, posy, 10 + velx, 2 + vely);
        double y = trans(5 + posx, 5 + posy, 10 + velx, 2 + vely);
        results.emplace_back(y);

        logger.LogData(posx, posy, velx, vely, y);
    }
    std::cout << "simulation finished in " << timer.toc() << " seconds, sample size: " << pos_samples.size() << " , " << vel_samples.size() << std::endl;

    return 0;
}