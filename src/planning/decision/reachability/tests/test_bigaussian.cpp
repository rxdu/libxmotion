#include <iostream>
#include <stdio.h>

// #include <gsl/gsl_rng.h>
// #include <gsl/gsl_randist.h>
#include "random/bigaussian_sampler.hpp"


using namespace xmotion;

int main()
{
    // const gsl_rng_type *T;
    // gsl_rng *r;

    // gsl_rng_env_setup();
    // T = gsl_rng_default;
    // r = gsl_rng_alloc(T);

    double x = 0, y = 0;
    BiGaussianSampler sampler(0.1, 0.5, 0);

    for (int i = 0; i < 10000; i++)
    {
        // gsl_ran_bivariate_gaussian(r, 0.1, 0.5, 0, &x, &y);
        sampler.Sample(&x, &y);
        // TODO(revive): record samples through telemetry (XM_* / MCAP)
        // instead of the removed ad-hoc CSV logger.
        (void)x;
        (void)y;
    }

    // gsl_rng_free(r);
    return 0;
}