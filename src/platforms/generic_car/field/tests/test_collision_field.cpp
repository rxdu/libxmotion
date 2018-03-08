#include <iostream>
#include <memory>

#include "field/collision_field.hpp"
#include "field/traffic_participant.hpp"
#include "field/threat_distribution.hpp"

#include "fastplot/fastplot.hpp"
#include "fastplot/field_plot.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    const int32_t fsize_x = 100;
    const int32_t fsize_y = 100;

    CollisionField cfield(fsize_x, fsize_y);
    cfield.SetOriginCoordinate(0,0);

    GaussianPositionVelocityThreat gau(50, 50, 1, 1);

    std::shared_ptr<TrafficParticipant> pt0 = std::make_shared<TrafficParticipant>(fsize_x, fsize_y);
    pt0->SetThreatDistribution(gau);

    cfield.AddTrafficParticipant(0, pt0);
    cfield.UpdateCollisionField();

    std::cout << "collision field created" << std::endl;

    ScalarFieldMatrix mat = cfield.GenerateFieldMatrix(0, 1, 0, 1, true);

    // plot surface
    // FastPlot::ShowFieldSurface(mat.x, mat.y, mat.z, true);
    // FastPlot::ShowMatrixAsImage(mat.z);
    FastPlot::ShowMatrixAsColorMap(mat.z);
    // FastPlot::ShowFieldContour(mat.x, mat.y, mat.z, true);
    
    return 0;
}