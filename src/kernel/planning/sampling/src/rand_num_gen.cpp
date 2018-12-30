/* 
 * rand_num_gen.cpp
 * 
 * Created on: Dec 29, 2018 10:32
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "sampling/details/sampler/rand_num_gen.hpp"

namespace librav
{
double RandNumGen::Uniform()
{
    return uniform_rdist_(generator_);
}

double RandNumGen::UniformReal(double lower_bound, double upper_bound)
{
    assert(lower_bound <= upper_bound);
    return (upper_bound - lower_bound) * uniform_rdist_(generator_) + lower_bound;
}

int RandNumGen::UniformInt(int lower_bound, int upper_bound)
{
    auto r = (int)floor(UniformReal((double)lower_bound, (double)(upper_bound) + 1.0));
    return (r > upper_bound) ? upper_bound : r;
}

bool RandNumGen::UniformBool()
{
    return uniform_rdist_(generator_) <= 0.5;
}

double RandNumGen::Gaussian()
{
    return normal_dist_(generator_);
}

double RandNumGen::Gaussian(double mean, double stddev)
{
    return normal_dist_(generator_) * stddev + mean;
}

double RandNumGen::HalfNormalReal(double r_min, double r_max, double focus)
{
    assert(r_min <= r_max);

    const double mean = r_max - r_min;
    double v = Gaussian(mean, mean / focus);

    if (v > mean)
        v = 2.0 * mean - v;
    double r = v >= 0.0 ? v + r_min : r_min;
    return r > r_max ? r_max : r;
}

int RandNumGen::HalfNormalInt(int r_min, int r_max, double focus)
{
    auto r = (int)floor(HalfNormalReal((double)r_min, (double)(r_max) + 1.0, focus));
    return (r > r_max) ? r_max : r;
}

void RandNumGen::Quaternion(double value[4])
{
    double x0 = uniform_rdist_(generator_);
    double r1 = sqrt(1.0 - x0), r2 = sqrt(x0);
    double t1 = 2.0 * M_PI * uniform_rdist_(generator_),
           t2 = 2.0 * M_PI * uniform_rdist_(generator_);
    double c1 = cos(t1), s1 = sin(t1);
    double c2 = cos(t2), s2 = sin(t2);
    value[0] = s1 * r1;
    value[1] = c1 * r1;
    value[2] = s2 * r2;
    value[3] = c2 * r2;
}

void RandNumGen::EulerRPY(double value[3])
{
    value[0] = M_PI * (-2.0 * uniform_rdist_(generator_) + 1.0);
    value[1] = std::acos(1.0 - 2.0 * uniform_rdist_(generator_)) - M_PI / 2.0;
    value[2] = M_PI * (-2.0 * uniform_rdist_(generator_) + 1.0);
}

void RandNumGen::SetLocalSeed(std::uint_fast32_t localSeed)
{
    // Store the seed
    local_seed_ = localSeed;

    // Change the generator's seed
    generator_.seed(local_seed_);

    // Reset the distributions used by the variate generators, as they can cache values
    uniform_rdist_.reset();
    normal_dist_.reset();
}
} // namespace librav