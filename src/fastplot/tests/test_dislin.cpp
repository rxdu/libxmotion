#include <iostream>
#include <cmath>
#include "discpp.h"

// double xray[50], yray[50], zmat[50][50];

// int main ()
// { int n = 50, i, j;
//   double  fpi = 3.14159 / 180.0, step, x, y;
//   double  zlev;
//   Dislin g;

//   step = 360.0/ (n - 1);

//   for (i = 0; i < n; i++)
//   { xray[i] = i * step;
//     yray[i] = i * step;
//   }

//   for (i = 0; i < n; i++)
//   { for (j = 0; j < n; j++)
//     { x = xray[i] * fpi;
//       y = yray[j] * fpi;
//       zmat[i][j] = 2 * sin (x) * sin (y);
//     }
//   }

//   g.scrmod ("revers");
//   g.setpag ("USAL");
//   g.sclfac(0.5);
//   g.metafl ("cons");
//   g.disini ();
//   g.complx ();
//   g.pagera ();

//   g.titlin ("Contour Plot", 1);
//   g.titlin ("F(X,Y) = 2 * SIN(X) * SIN(Y)", 3);

//   g.name   ("X-axis", "x");
//   g.name   ("Y-axis", "y");

//   g.intax  ();
//   g.axspos (450, 2670);
//   g.graf   (0.0, 360.0, 0.0, 90.0, 0.0, 360.0, 0.0, 90.0);

//   g.height (30);
//   for (i = 0; i < 9; i++)
//   { zlev = -2.0 + i * 0.5;
//     g.setclr ((i + 1) * 25);
//     if (i == 4)
//       g.labels ("none", "contur");
//     else
//       g.labels ("float", "contur");

//     g.contur  (xray, n, yray, n,(double *) zmat, zlev);
//   }

//   g.height (50);
//   g.color  ("fore");
//   g.title  ();
//   g.disfin ();
//   return 0;
// }


double zmat[50][50], xray[50], yray[50];

int main ()
{ int n = 50 ,i, j;
  double fpi = 3.1415927 / 180.0, step, x, y;
  const char *ctit1 = "Shaded Surface Plot",
             *ctit2 = "F(X,Y) = 2*SIN(X)*SIN(Y)";
  Dislin g;

  step = 360.0/ (n - 1);
  for (i = 0; i < n; i++)
  { x = i * step;
    xray[i] = x;
    for (j = 0; j < n; j++)
    { y = j * step;
      yray[j] = y;
      zmat[i][j] = 2 * sin (x * fpi) * sin (y * fpi);
    }
  }

  g.scrmod ("revers");
  g.setpag ("da4p");
  g.metafl ("cons");
  g.disini ();
  g.pagera ();
  g.complx ();
  g.axspos (200, 2600);
  g.axslen (1800, 1800);

  g.name   ("X-axis", "x");
  g.name   ("Y-axis", "y");
  g.name   ("Z-axis", "z");

  g.titlin (ctit1, 2);
  g.titlin (ctit2, 4);

  g.view3d (-5.0, -5.0, 4.0, "abs");
  g.graf3d (0.0, 360.0, 0.0, 90.0, 0.0, 360.0, 0.0, 90.0,
            -3.0, 3.0, -3.0, 1.0);
  g.height (50);
  g.title  ();

  g.shdmod ("smooth", "surface"); 
  g.surshd (xray, n, yray, n, (double *) zmat);
  g.disfin ();
  return 0;
}