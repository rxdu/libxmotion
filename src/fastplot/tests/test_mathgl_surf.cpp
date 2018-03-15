#include <iostream>
#include <memory>

#include "fastplot/mathgl_surf.hpp"

using namespace librav;

int main()
{
    MathGLSurf surf;
    MathGLPlot::Run(&surf);

    return 0;
}
