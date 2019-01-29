# include <Eigen/Dense>
# include "figure.hpp"

int main(){
  
  std::vector<double> t(100), y(100), z(100);
  for (int i = -50; i < 50; ++i){
    t[i + 50] = i;
    y[i + 50] = i*i - 2*i;
    z[i + 50] = -i*std::exp(std::sqrt(std::abs(i))*std::sin(0.5*i));
  }

  mgl::Figure fig;
  fig.plot(t, y, "b0").label("f(x)");
  fig.plot(t, z, "r0").label("g(x)");
  fig.title("Some functions");
  fig.legend();
  fig.save("plot-1.eps");

  // to compare what normally must be done for the same plot 
  /*
  mglData td(t.data(), t.size()),
          yd(y.data(), y.size()),
          zd(z.data(), z.size());

  mglGraph gr;
  gr.SetFontSizePT(8);
  gr.SubPlot(1,1,0,"<_");
  gr.SetRanges( td.Minimal(), td.Maximal(), std::min(yd.Minimal(), zd.Minimal()), std::max(yd.Maximal(), zd.Maximal()) );
  gr.Plot(td, yd, "b0");
  gr.AddLegend("f(x)","b0");
  gr.Plot(td, zd, "r0");
  gr.AddLegend("g(x)","r0");
  gr.Legend();
  gr.Title("Some functions");
  gr.Label('y', "y Axis");
  gr.Grid("","h");
  gr.Axis();
  gr.WriteEPS("plot-alt.eps");
  */

  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(500, 1, 20);
  Eigen::VectorXd fx = (10*x.array().exp()).matrix();

  mgl::Figure fig2;
  fig2.setlog(false, true);
  fig2.ylabel("Some long y Axis label");
  fig2.xlabel("Some long x Axis label");
  fig2.plot(x, fx, "g0");
  fig2.title("Some title");
  fig2.save("plot-2.eps");

  return 0;
}
  
