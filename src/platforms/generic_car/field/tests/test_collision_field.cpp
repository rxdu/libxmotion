#include <iostream>

#include "field/collision_field.hpp"

#include "fastplot/field_plot.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    const int32_t fsize_x = 300;
    const int32_t fsize_y = 300;

    CollisionField cfield(fsize_x, fsize_y);
    cfield.SetOriginCoordinate(150,150);

    std::cout << "collision field created" << std::endl;

    ScalarFieldMatrix mat = cfield.GenerateFieldMatrix(0, 1, 0, 1, true);

    // plot surface
    FieldPlot fplot(50, 50);
    fplot.SetWrapScale(4.0);
    fplot.ShowField(mat.x, mat.y, mat.z, true);

    return 0;
}