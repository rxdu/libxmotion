#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    TrafficParticipant<GaussianPositionVelocityThreat> traff_part(100, 100);

    stopwatch::StopWatch timer;

    timer.tic();

    traff_part.SetParameters(50, 50, 2, -1, 15, 15);
    ScalarFieldMatrix mat = traff_part.GenerateFieldMatrix(0, 1, 0, 1, true);

    std::cout << "time elapsed: " << timer.toc() << std::endl;

    // plot surface
    LightViz::ShowMatrixAsColorMap(mat.z);

    return 0;
}