#include <iostream>

#include "threat_field/information_field.hpp"

#include "fastplot/field_plot.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    const int32_t fsize_x = 301;
    const int32_t fsize_y = 301;

    InformationField cfield(fsize_x, fsize_y);
    cfield.SetOriginCoordinate(150,150);
    cfield.LoadEgoCenteredBasisPattern(50, M_PI/4);
    // cfield.LoadUniformBasisPattern(50,50);
    cfield.UpdateInformationField();

    std::cout << "information field created" << std::endl;

    ScalarFieldMatrix mat = cfield.GenerateFieldMatrix(0, 1, 0, 1, true);

    // plot surface
    FastPlot::ShowFieldSurface(mat.x, mat.y, mat.z, true);
    // FastPlot::ShowFieldContour(mat.x, mat.y, mat.z, true);
    
    return 0;
}