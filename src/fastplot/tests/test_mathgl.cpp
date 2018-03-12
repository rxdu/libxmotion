#include <iostream>
#include <memory>

#include "mgl2/mgl.h"
#include <mgl2/glut.h>

void mgls_prepare2d(mglData *a, mglData *b = 0, mglData *v = 0)
{
    register long i, j, n = 50, m = 40, i0;
    if (a)
        a->Create(n, m);
    if (b)
        b->Create(n, m);
    if (v)
    {
        v->Create(9);
        v->Fill(-1, 1);
    }
    mreal x, y;
    for (i = 0; i < n; i++)
        for (j = 0; j < m; j++)
        {
            x = i / (n - 1.);
            y = j / (m - 1.);
            i0 = i + n * j;
            if (a)
                a->a[i0] = 0.6 * sin(2 * M_PI * x) * sin(3 * M_PI * y) + 0.4 * cos(3 * M_PI * x * y);
            if (b)
                b->a[i0] = 0.6 * cos(2 * M_PI * x) * cos(3 * M_PI * y) + 0.4 * cos(3 * M_PI * x * y);
        }
}

int sample(mglGraph *gr)
{
    mglData a;
    mgls_prepare2d(&a);
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
    mglData x(50, 40), y(50, 40), z(50, 40);
    gr->Fill(x, "0.8*sin(pi*x)*sin(pi*(y+1)/2)");
    gr->Fill(y, "0.8*cos(pi*x)*sin(pi*(y+1)/2)");
    gr->Fill(z, "0.8*cos(pi*(y+1)/2)");
    gr->Rotate(50, 60);
    gr->Box();
    gr->Surf(x, y, z, "BbwrR");
    return 0;
}

void mgls_prepare1d(HMDT y, HMDT y1 = 0, HMDT y2 = 0, HMDT x1 = 0, HMDT x2 = 0)
{
    register long i, n = 50;
    if (y)
        mgl_data_create(y, n, 3, 1);
    if (x1)
        mgl_data_create(x1, n, 1, 1);
    if (x2)
        mgl_data_create(x2, n, 1, 1);
    if (y1)
        mgl_data_create(y1, n, 1, 1);
    if (y2)
        mgl_data_create(y2, n, 1, 1);
    mreal xx;
    for (i = 0; i < n; i++)
    {
        xx = i / (n - 1.);
        if (y)
        {
            mgl_data_set_value(y, 0.7 * sin(2 * M_PI * xx) + 0.5 * cos(3 * M_PI * xx) + 0.2 * sin(M_PI * xx), i, 0, 0);
            mgl_data_set_value(y, sin(2 * M_PI * xx), i, 1, 0);
            mgl_data_set_value(y, cos(2 * M_PI * xx), i, 2, 0);
        }
        if (y1)
            mgl_data_set_value(y1, 0.5 + 0.3 * cos(2 * M_PI * xx), i, 0, 0);
        if (y2)
            mgl_data_set_value(y2, 0.3 * sin(2 * M_PI * xx), i, 0, 0);
        if (x1)
            mgl_data_set_value(x1, xx * 2 - 1, i, 0, 0);
        if (x2)
            mgl_data_set_value(x2, 0.05 + 0.03 * cos(2 * M_PI * xx), i, 0, 0);
    }
}

int main()
{
    // mglGraph *gr = new mglGraph();
    mglGLUT * gr = new mglGLUT(sample, "MathGL examples");

    sample(gr);

    // delete gr;

    return gr->Run();

    // std::shared_ptr<mglGraph> gr = std::make_shared<mglGraph>();

    // mglData y;
    // mgls_prepare1d(&y);
    // gr->SetOrigin(0, 0, 0);
    // gr->SubPlot(2, 2, 0, "");
    // gr->Title("Plot plot (default)");
    // gr->Box();
    // gr->Plot(y);

    // gr->SubPlot(2, 2, 2, "");
    // gr->Title("'!' style; 'rgb' palette");
    // gr->Box();
    // gr->Plot(y, "o!rgb");

    // gr->SubPlot(2, 2, 3, "");
    // gr->Title("just markers");
    // gr->Box();
    // gr->Plot(y, " +");

    // gr->SubPlot(2, 2, 1);
    // gr->Title("3d variant");
    // gr->Rotate(50, 60);
    // gr->Box();
    // mglData yc(30), xc(30), z(30);
    // z.Modify("2*x-1");
    // yc.Modify("sin(pi*(2*x-1))");
    // xc.Modify("cos(pi*2*x-pi)");
    // gr->Plot(xc, yc, z, "rs");

    return 0;
}