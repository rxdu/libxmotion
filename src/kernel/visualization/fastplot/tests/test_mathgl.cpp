#include "mgl2/mgl.h"
#include <mgl2/fltk.h>

int sample(mglGraph *gr)
{
    register long i, j, n = 50, m = 40, i0;

    mglData a;
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

    gr->SubPlot(2, 2, 0);
    gr->Title("Surf plot (default)");
    gr->Light(true);
    gr->Rotate(50, 60);
    gr->Box();
    gr->Surf(a);

    gr->SubPlot(2, 2, 1);
    gr->Title("'\\#' style; meshnum 10");
    gr->Rotate(50, 60);
    gr->Box();
    gr->Surf(a, "#", "meshnum 10");

    gr->SubPlot(2, 2, 2);
    gr->Title("'.' style");
    gr->Rotate(50, 60);
    gr->Box();
    gr->Surf(a, ".");

    gr->SubPlot(2, 2, 3);
    gr->Title("parametric form");
    mglData x2(50, 40), y2(50, 40), z2(50, 40);
    gr->Fill(x2, "0.8*sin(pi*x)*sin(pi*(y+1)/2)");
    gr->Fill(y2, "0.8*cos(pi*x)*sin(pi*(y+1)/2)");
    gr->Fill(z2, "0.8*cos(pi*(y+1)/2)");
    gr->Rotate(50, 60);
    gr->Box();
    gr->Surf(x2, y2, z2, "BbwrR");
    return 0;
}

int main()
{
    // mglGLUT *gr = new mglGLUT(sample, "MathGL examples");
    mglFLTK *gr = new mglFLTK(sample, "MathGL examples");

    gr->Run();
    delete gr;

    return 0;
}