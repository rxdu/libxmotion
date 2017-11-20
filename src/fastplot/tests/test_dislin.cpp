#include <iostream>
#include <cmath>
#include "discpp.h"

double zmat[50][50];

int main()
{
    int n = 50, i, j;
    double fpi = 3.1415927 / 180.0, step, x, y;
    const char *ctit1 = "Surface Plot (SURMAT)",
               *ctit2 = "F(X,Y) = 2*SIN(X)*SIN(Y)";
    Dislin g;

    step = 360.0 / (n - 1);
    for (i = 0; i < n; i++)
    {
        x = i * step;
        for (j = 0; j < n; j++)
        {
            y = j * step;
            zmat[i][j] = 2 * sin(x * fpi) * sin(y * fpi);
        }
    }

    g.scrmod("revers");
    g.setpag("da4p");
    g.metafl("cons");
    g.disini();
    g.pagera();
    g.complx();
    g.axspos(200, 2600);
    g.axslen(1800, 1800);

    g.name("X-axis", "x");
    g.name("Y-axis", "y");
    g.name("Z-axis", "z");

    g.titlin(ctit1, 2);
    g.titlin(ctit2, 4);

    g.view3d(-5.0, -5.0, 4.0, "abs");
    g.graf3d(0.0, 360.0, 0.0, 90.0, 0.0, 360.0, 0.0, 90.0,
             -3.0, 3.0, -3.0, 1.0);
    g.height(50);
    g.title();

    g.color("green");
    g.surmat((double *)zmat, 50, 50, 1, 1);
    g.disfin();
    return 0;
}
