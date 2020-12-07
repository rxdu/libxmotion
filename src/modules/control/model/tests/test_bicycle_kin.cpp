#include <iostream>

#include "model/bicycle_model.hpp"

using namespace robotnav;
using namespace asc;

int main()
{
    state_t x = {0.0, 8.0};

    double t = 0.0;
    double dt = 0.001;
    double t_end = 10.0;
    BicycleKinematics::control_type u = {0.1, 0};

    RK4 integrator;
    Recorder recorder;

    while (t <= t_end)
    {
        recorder({t, x[0], x[1], u.a, u.delta});
        integrator(BicycleKinematics(u), x, t, dt);
    }

    recorder.csv("vehicle_longitudinal", {"t", "x0", "x1", "u"});

    std::cout << "final state: " << x[0] << " , " << x[1] << std::endl;

    return 0;
}