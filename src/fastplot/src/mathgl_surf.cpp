/* 
 * mathgl_surf.cpp
 * 
 * Created on: Mar 14, 2018 23:17
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "fastplot/mathgl_surf.hpp"

using namespace librav;

int MathGLSurf::Draw(mglGraph *gr)
{
    std::cout << "run sample" << std::endl;
    register long i, j, n = 50, m = 40, i0;

    mglData a;
    // mgls_prepare2d(&a);
    a.Create(n, m);
    mreal x, y;
    for (i = 0; i < n; i++)
        for (j = 0; j < m; j++)
        {
            x = i / (n - 1.);
            y = j / (m - 1.);
            i0 = i + n * j;
            a.a[i0] = 0.6 * sin(2 * M_PI * x) * sin(3 * M_PI * y) + 0.4 * cos(3 * M_PI * x * y);
        }

    // gr->SubPlot(2, 2, 0);
    gr->Title("Surf plot (default)");
    gr->Light(true);
    gr->Rotate(50, 60);
    gr->Box();
    gr->Axis();
    gr->Surf(a);

    return 0;
}