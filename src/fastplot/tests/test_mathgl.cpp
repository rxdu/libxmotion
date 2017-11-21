
#include "mgl2/mgl.h"
#include "mgl2/font.h"
#include "mgl2/eval.h"

int main()
{
    // mglData dat(30, 40); // data to for plotting
    // for (long i = 0; i < 30; i++)
    //     for (long j = 0; j < 40; j++)
    //         dat.a[i + 30 * j] = 1 / (1 + (i - 15) * (i - 15) / 225. + (j - 20) * (j - 20) / 400.);

    mglGraph gr;
    // gr.Title("Mesh plot");
    // gr.Rotate(50, 60);
    // gr.Box();
    // gr.Mesh(dat);
    gr.FPlot("sin(pi*x)");
    gr.WriteFrame("test.png");

    return 0;
}