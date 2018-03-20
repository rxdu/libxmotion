#include "reachability/vehicle_dynamics.hpp"

using namespace librav;
using namespace asc;

int main()
{
    state_t x = {0.0, 8.0};
    double t = 0.0;
    double dt = 0.001;
    double t_end = 10.0;
    double u = 0.1;

    RK4 integrator;
    Recorder recorder;

    while (t < t_end)
    {
        recorder({t, x[0], x[1], u});
        integrator(LongitudinalDynamics(u), x, t, dt);
    }

    recorder.csv("vehicle_longitudinal", {"t", "x0", "x1", "u"});

    return 0;
}