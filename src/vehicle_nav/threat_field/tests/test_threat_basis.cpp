#include "threat_field/threat_basis.hpp"
#include "threat_field/threat_distribution.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    ThreatBasis basis(100, 100);

    stopwatch::StopWatch timer;
    timer.tic();
    GaussianPositionThreat gau(50, 50, 10, 3);
        // GaussianPositionThreat gau(50, 50, 5, 3);

    basis.SetThreatBasisDistribution(gau);
    // GaussianPositionVelocityThreat gau(100, 100, 1, 0, 5);
    // basis.SetThreatBasisDistribution(gau);
    ScalarFieldMatrix mat = basis.GenerateFieldMatrix(0, 1, 0, 1, true);
    std::cout << "time elapsed: " << timer.toc() << std::endl;

    // plot surface
    // LightViz::ShowMatrixAsImage(mat.z);
    LightViz::ShowMatrixAsColorMap(mat.z);

    return 0;
}