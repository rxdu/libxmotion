
#include <iostream>
#include <cmath>

#include "fastplot/field_plot.hpp"

using namespace librav;

int main()
{
  // prepare data
  Eigen::VectorXf x(20);
  Eigen::VectorXf y(20);
  Eigen::MatrixXf z(20, 20);

  for (int i = 0; i < 20; i++)
  {
    x(i) = 1 + 0.5*i;
  }

  for (int j = 0; j < 20; j++)
  {
    y(j) = 1 + j;
  }

  for (int i = 0; i < 20; i++)
    for (int j = 0; j < 20; j++)
    {
      z(i,j) = sinf(x(i)) + cosf(y(j));
    }

  // plot surface
  FieldPlot fplot(20,20);
  fplot.ShowField(x,y,z);
  // splot.SaveSurfaceToFile(x,y,z,"test_surf.png",1024,768);

  return 0;
}