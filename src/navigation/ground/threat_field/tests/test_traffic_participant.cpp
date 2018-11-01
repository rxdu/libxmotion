#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"

#include "lightviz/lightviz.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    GaussianPositionVelocityThreat threat_model(50, 50, 2, -1, 15, 15);

    std::shared_ptr<TrafficParticipant> participant = std::make_shared<TrafficParticipant>(50, 50, 2, -1);
    participant->threat_func = threat_model;

    LightViz::ShowTrafficParticipant(participant);

    // plot surface
    // LightViz::ShowMatrixAsColorMap(mat.z);

    return 0;
}