#include <iostream>
#include <cmath>

#include "traffic_map/map_loader.hpp"

// #include "lightviz/lightviz.hpp"
#include "navviz/vehicle_draw.hpp"

using namespace librav;

int main()
{
    Polygon fp;
    fp.AddPoint(1.2 * 2, 0.9);
    fp.AddPoint(1.2 * 2, -0.9);
    fp.AddPoint(-1.2 * 2, -0.9);
    fp.AddPoint(-1.2 * 2, 0.9);

    std::vector<Polygon> polys;
    auto fp_start = fp.TransformRT(10, 5, 82 / 180.0 * M_PI);
    auto fp_final = fp.TransformRT(5, 8, 170.0 / 180.0 * M_PI);
    polys.push_back(fp_start);
    polys.push_back(fp_final);

    CvCanvas canvas(10);
    canvas.Resize(-50, 50, -50, 50);
    // canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);

    VehicleViz::DrawVehicle(canvas, fp);
    VehicleViz::DrawVehicle(canvas, fp, 5);
    VehicleViz::DrawVehicle(canvas, polys);

    canvas.Show();

    return 0;
}