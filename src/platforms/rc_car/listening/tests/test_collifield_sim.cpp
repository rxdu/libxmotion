#include <iostream>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "field/collision_field.h"
#include "traffic/traffic_sim.h"
#include "utility/stopwatch/stopwatch.h"

using namespace librav;

#define LOOP_PERIOD 1500

int main()
{
    // set up network first
    std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good())
    {
        std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
        return -1;
    }

    auto cfield = std::make_shared<CollisionField>(200, 200);

    std::cout << "started traffic sim" << std::endl;
    TrafficSim sim;

    sim.SetDuration(50);
    sim.SetStartTime(0);
    sim.SetStepSize(1);

    sim.vehicle_.SetPosition(Position2Dd(81,0));

    stopwatch::StopWatch timer;

    // simulation loop
    bool sim_finished = false;
    while (!sim_finished)
    {
        timer.tic();

        sim_finished = sim.UpdateTraffic();

        auto pos = sim.vehicle_.GetPosition();
        cfield->vehicle_field_->SetCarPosition(static_cast<int64_t>(pos.x),static_cast<int64_t>(pos.y));
        cfield->vehicle_field_->UpdateDistribution();
        cfield->CombineAllFields();
        librav_lcm_msgs::ScalarField_t msg = cfield->GenerateScalarFieldMsg();

        lcm->publish("ScalarField", &msg);
        std::cout << "scalar field msg published" << std::endl;

        timer.sleep_ms_util(LOOP_PERIOD);
    }

    return 0;
}