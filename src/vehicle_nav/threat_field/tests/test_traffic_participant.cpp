#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    GaussianPositionVelocityThreat threat_model(50, 50, 2, -1, 15, 15);

    TrafficParticipant participant;
    participant.threat_func = threat_model;

    // plot surface
    // LightViz::ShowMatrixAsColorMap(mat.z);

    return 0;
}