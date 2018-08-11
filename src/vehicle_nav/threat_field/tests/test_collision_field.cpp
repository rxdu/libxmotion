#include <iostream>
#include <memory>

#include "threat_field/collision_field.hpp"
#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"

// #include "fastplot/fastplot.hpp"
// #include "fastplot/field_plot.hpp"
#include "stopwatch/stopwatch.h"

#include "lightviz/lightviz.hpp"

using namespace librav;

int main()
{
    const int32_t fsize_x = 800;
    const int32_t fsize_y = 600;

    CollisionField cfield(fsize_x, fsize_y);
    cfield.SetOriginCoordinate(0, 0);

    // add traffic participant
    auto pt0 = std::make_shared<CollisionField::TrafficParticipantType>(fsize_x, fsize_y);
    pt0->SetParameters(220, 150, 1, -1, 15, 15);

    cfield.AddTrafficParticipant(0, pt0);

    // update collision field before use
    cfield.UpdateCollisionField();

    std::cout << "collision field created" << std::endl;

    ScalarFieldMatrix mat = cfield.GenerateFieldMatrix(0, 1, 0, 1, true);

    // plot surface
    // FastPlot::ShowFieldSurface(mat.x, mat.y, mat.z, true);
    // FastPlot::ShowFieldContour(mat.x, mat.y, mat.z, true);
    // FastPlot::ShowMatrixAsImage(mat.z);
    // LightViz::ShowMatrixAsImage(mat.z);
    LightViz::ShowMatrixAsColorMap(mat.z);

    return 0;
}