#include <iostream>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "field/collision_field.hpp"
#include "traffic/traffic_sim.hpp"
#include "stopwatch/stopwatch.h"

#include "fastplot/field_plot.hpp"

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

    sim.vehicle_.SetPosition(Position2Dd(81, 0));

    stopwatch::StopWatch timer;
    FieldPlot surf_plot(200, 200);
    surf_plot.SetCameraPosition(0, 100, 100);

    // simulation loop flags
    bool sim_finished = false;
    bool first_run = true;

    // add vehicle field
    auto vehicle_field1 = std::make_shared<VehicleField>(200, 200);
    cfield->AddVehicleField(1, vehicle_field1);

    // start simulation
    while (!sim_finished)
    {
        timer.tic();

        sim_finished = sim.UpdateTraffic();

        auto pos = sim.vehicle_.GetPosition();
        vehicle_field1->SetCarPosition(static_cast<int64_t>(pos.x), static_cast<int64_t>(pos.y));
        cfield->UpdateCollisionField();

        // plot surface
        ScalarFieldMatrix mat = cfield->GenerateFieldMatrix(0, 1, 0, 1.5);
        surf_plot.ShowFieldFrame(mat.x, mat.y, mat.z, false);

        // librav_lcm_msgs::ScalarField_t msg = cfield->GenerateScalarFieldMsg();
        // lcm->publish("ScalarField", &msg);
        // std::cout << "scalar field msg published" << std::endl;

        timer.sleep_until_ms(LOOP_PERIOD);
    }

    return 0;
}