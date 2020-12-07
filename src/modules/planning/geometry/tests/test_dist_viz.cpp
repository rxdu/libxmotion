#include "canvas/cv_draw.hpp"
#include "geometry/polygon.hpp"
#include "coreviz/geometry_draw.hpp"

using namespace robotnav;

// copied here to avoid dependency from a higher-level package
class GaussianPositionThreat
{
  public:
    GaussianPositionThreat(double pos_x, double pos_y, double sigma1, double sigma2) : pos_x_(pos_x),
                                                                                       pos_y_(pos_y),
                                                                                       sigma_1_(sigma1),
                                                                                       sigma_2_(sigma2)
    {
        // update coeff1_ and coeff2_
        SetParameters(pos_x, pos_y, sigma1, sigma2);
    }

    void SetParameters(double pos_x, double pos_y, double sigma1, double sigma2)
    {
        pos_x_ = pos_x;
        pos_y_ = pos_y;
        sigma_1_ = sigma1;
        sigma_2_ = sigma2;

        coeff1_ = (2 * M_PI * sigma_1_ * sigma_2_);
        coeff2_ = -(2 * sigma_1_ * sigma_1_);
        coeff3_ = -(2 * sigma_2_ * sigma_2_);
    }
    double operator()(double x, double y)
    {
        double x_err = x - pos_x_;
        double y_err = y - pos_y_;

        double val = std::exp(x_err * x_err / coeff2_ + y_err * y_err / coeff3_) / coeff1_;

        return val;
    }

  private:
    double pos_x_ = 0;
    double pos_y_ = 0;
    double sigma_1_ = 1;
    double sigma_2_ = 1;

    double coeff1_ = 0;
    double coeff2_ = 1;
    double coeff3_ = 1;
};

int main()
{
    CartesianCanvas canvas(10);
    canvas.SetupCanvas(0, 100, 0, 80, CvDrawColors::jet_colormap_lowest);

    GeometryViz gdraw(canvas);

    GaussianPositionThreat threat_model(50, 60, 2, 2);

    gdraw.DrawDistribution(50, 60, 20, 20, threat_model);

    CvDraw::ShowImage(canvas.paint_area, "test field viz");

    return 0;
}