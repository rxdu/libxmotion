#include <iostream>
#include <memory>

#include "prediction/collision_field.hpp"
#include "prediction/traffic_participant.hpp"
#include "prediction/threat_distribution.hpp"

#include "stopwatch.hpp"

#define ENABLE_VIZ

#ifdef ENABLE_VIZ
#include "lightviz/navviz.hpp"
#endif

using namespace robotnav;

int main()
{
    // add traffic participant
    // GaussianPositionThreat threat_model(0, 0, 1.8/2, 4.8/2);
    GaussianPositionVelocityThreat threat_model(0, 0, 1, 1, 1.8/2, 4.8/2);

    std::shared_ptr<TrafficParticipant> participant = std::make_shared<TrafficParticipant>(0, 0, 1, 1);
    participant->threat_func = threat_model;

    std::cout << "collision field created" << std::endl;

    Polygon fp;
    fp.AddPoint(0.9, 2.4);
    fp.AddPoint(-0.9, 2.4);
    fp.AddPoint(-0.9, -2.4);
    fp.AddPoint(0.9, -2.4);

#ifdef ENABLE_VIZ
    // plot surface
    LightViz::ShowTrafficParticipantThreat(participant, fp, 15);//, "true_threat", true);
#endif

    return 0;
}