/*
  Test program for MatPlot

  Dag Lindbo, dag@csc.kth.se
*/

#include "matplot.h"

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace ublas = boost::numeric::ublas;
typedef ublas::vector<double> Vector;
typedef ublas::matrix<double> Matrix;

using namespace std;
using namespace matplot;

// parameters for 2D plot
const int NN = 40;
const double t_low = 0;
const double t_upp = 5;
const double dt = (t_upp-t_low)/(NN-1);

// parameters for 3D stuff
const int Nx = 200;
const int Ny = 250;
const double x_low = -2.5;
const double x_upp = 1.5;
const double y_low = -2.5;
const double y_upp = 2.5;
const double dx = (x_upp-x_low)/(Nx-1);
const double dy = (y_upp-y_low)/(Ny-1);

#define G(x,y) exp(-(x*x+y*y))*sin(x)*cos(y);

// parameters for quiver plot
const int NNx = 20;
const int NNy = 37;
const double xx_low = 0;
const double xx_upp = 1;
const double yy_low = 0;
const double yy_upp = 1.5;
const double dxx = (xx_upp-xx_low)/(NNx-1);
const double dyy = (yy_upp-yy_low)/(NNy-1);

int main(void)
{
  int i, j;

  // EXAMPLE: Plot_2D =======================================
  Vector x1(NN);
  Vector y1(NN);
  Vector x2(NN);
  Vector y2(NN);

  double t = t_low;

  for (i=0; i<NN; i++)
    {
      x1(i) = cos(t)*t;
      y1(i) = sin(t);
      x2(i) = sin(t)*t+1;
      y2(i) = cos(t)*(0.1-t);
      t+=dt;
    }

  Plot2D_VTK p_2d("x(t)", "y(t)", 800, 600);

  double color1[3] =
    { 0.0, 0.0, 1.0 };
  double color2[3] =
    { 1.0, 0.0, 0.0 };

  p_2d.plot(x1, y1, color1, "-");
  p_2d.plot(x2, y2, color2, ".-");
  p_2d.show();
  //p_2d.draw_to_png("plot_2d.png");

  // EXAMPLE: Contour =======================================

  Vector x(Nx);
  Vector y(Ny);
  Matrix z(Nx, Ny);

  for (i=0; i<Nx; i++)
    x(i) = x_low+i*dx;

  for (i=0; i<Ny; i++)
    y(i) = y_low + i*dy;

  for (i=0; i<Nx; i++)
    for (j=0; j<Ny; j++)
      z(i, j) = G(x(i),y(j));

  Contour_VTK p_cont(800, 600);
  p_cont.contour(x, y, z, true, 20);
  //p_cont.contour_to_file(x,y,z,true,20,"contours.png");

  // EXAMPLE: Surf ==========================================

  double cam[3] =
    { -15.0, 10.0, 12.0 };
  double focal[3] =
    { 0, 0, 0 };
  Surf_VTK p_surf(800, 600);
  //p_surf.surf(x,y,z);
  //p_surf.surf(x,y,z,false);
  p_surf.surf(x, y, z, true, cam, focal);
  //p_surf.surf_to_file(x,y,z,true,"surf.png",cam,focal);

  // Example: Quiver ========================================
  Vector xx(NNx);
  Vector yy(NNy);
  Matrix u(NNx, NNy);
  Matrix v(NNx, NNy);

  for (i=0; i<NNx; i++)
    xx(i) = xx_low+i*dxx;

  for (i=0; i<NNy; i++)
    yy(i) = yy_low + i*dyy;

  for (i=0; i<NNx; i++)
    for (j=0; j<NNy; j++)
      {
   	u(i, j) = -yy(j);
   	v(i, j) = xx(i);
      }

  Quiver_VTK p_quiver(800, 600);
  p_quiver.quiver(xx, yy, u, v, 0.1);
  //p_quiver.quiver_to_file(xx,yy,u,v,0.1,"quiver.png");

  return 0;
}
