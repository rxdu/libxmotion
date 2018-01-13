#include <iostream>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "field/vehicle_field.h"
#include "traffic/traffic_sim.h"
#include "stopwatch/stopwatch.h"

using namespace librav;

#define LOOP_PERIOD 500

int main()
{
    // set up network first
    std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good())
    {
        std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
        return -1;
    }

    auto vfield = std::make_shared<VehicleField>(200, 200);

    std::cout << "started traffic sim" << std::endl;
    TrafficSim sim;

    sim.SetDuration(50);
    sim.SetStartTime(0);
    sim.SetStepSize(1);

    sim.vehicle_.SetPosition(Position2Dd(0,20));

    stopwatch::StopWatch timer;

    // simulation loop
    bool sim_finished = false;
    while (!sim_finished)
    {
        timer.tic();

        sim_finished = sim.UpdateTraffic();

        auto pos = sim.vehicle_.GetPosition();
        vfield->SetCarPosition(static_cast<int64_t>(pos.x),static_cast<int64_t>(pos.y));
        vfield->UpdateDistribution();
        librav_lcm_msgs::ScalarField_t msg = vfield->GenerateScalarFieldMsg();

        lcm->publish("ScalarField", &msg);
        std::cout << "scalar field msg published" << std::endl;

        timer.sleep_util_ms(LOOP_PERIOD);
    }

    return 0;
}