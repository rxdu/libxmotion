#include <iostream>
#include <memory>

#include "threat_field/collision_field.hpp"
#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"

// #include "fastplot/fastplot.hpp"
// #include "fastplot/field_plot.hpp"
#include "stopwatch/stopwatch.h"

#include "lightviz/lightviz.hpp"
#include "vehicle_viz/vehicle_viz.hpp"

using namespace librav;

int main()
{

    // add traffic participant
    GaussianPositionVelocityThreat threat_model(40, 40, 2, -1, 1, 1);
    std::shared_ptr<TrafficParticipant> participant = std::make_shared<TrafficParticipant>(40, 40, 2, -1);
    participant->threat_func = threat_model;

    GaussianPositionVelocityThreat threat_model2(30, 20, 2, -1, 1, 1);
    std::shared_ptr<TrafficParticipant> participant2 = std::make_shared<TrafficParticipant>(30, 25, 2, -1);
    participant2->threat_func = threat_model2;

    LightViz::ShowTrafficParticipant(participant);

    std::shared_ptr<CollisionField> cfield = std::make_shared<CollisionField>(0, 80, 0, 60);
    cfield->AddTrafficParticipant(0, participant);
    cfield->AddTrafficParticipant(1, participant2);

    LightViz::ShowCollisionField(cfield);

    std::cout << "collision field created" << std::endl;

    return 0;
}