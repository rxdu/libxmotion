#include <iostream>
#include <memory>

#include "threat_field/collision_field.hpp"
#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"

// #include "fastplot/fastplot.hpp"
// #include "fastplot/field_plot.hpp"
#include "stopwatch/stopwatch.h"

#include "lightviz/lightviz.hpp"
#include "traffic_viz/traffic_viz.hpp"

using namespace librav;

int main()
{
    std::shared_ptr<CollisionField> cfield = std::make_shared<CollisionField>(0, 80, 0, 60);

    // add traffic participant
    GaussianPositionVelocityThreat threat_model(50, 50, 2, -1, 1, 1);

    std::shared_ptr<TrafficParticipant> participant = std::make_shared<TrafficParticipant>(50, 50, 2, -1);
    participant->threat_func = threat_model;

    cfield->AddTrafficParticipant(0, participant);

    std::cout << "collision field created" << std::endl;

    // plot surface
    // LightViz::ShowFieldDistribution(threat_model.GetCenterPositionX(), threat_model.GetCenterPositionY(), participant->threat_func);
    LightViz::ShowTrafficParticipant(participant);
    LightViz::ShowCollisionField(cfield);

    return 0;
}