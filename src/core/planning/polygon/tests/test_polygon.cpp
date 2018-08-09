#include <iostream>
#include "polygon/polyline.hpp"
#include "polygon/polygon.hpp"

using namespace librav;

int main()
{
    Polyline polyline;
    polyline.AddPoint(391, 374);
    polyline.AddPoint(240, 431);
    polyline.AddPoint(252, 340);
    polyline.AddPoint(374, 320);
    
    polyline.PrintInfo();

    Polygon polygon;
    polygon.AddPoint(391, 374);
    polygon.AddPoint(240, 431);
    polygon.AddPoint(252, 340);
    polygon.AddPoint(374, 320);
    polygon.AddPoint(289, 214);
    polygon.AddPoint(134, 390);
    polygon.AddPoint(68, 186);
    polygon.AddPoint(154, 259);
    polygon.AddPoint(161, 107);
    polygon.AddPoint(435, 108);
    polygon.AddPoint(208, 148);
    polygon.AddPoint(295, 160);
    polygon.AddPoint(421, 212);
    polygon.AddPoint(441, 303);

    // std::cout << polygon << std::endl;

    polygon.PrintInfo();
    std::cout << "is simple: " << polygon.IsSimple() << std::endl;
    std::cout << "is convex: " << polygon.IsConvex() << std::endl;
    std::cout << "pt (200,180) is inside: " << polygon.CheckInside(Polygon::Point(200, 180)) << std::endl;
    std::cout << "pt (391,374) is inside: " << polygon.CheckInside(Polygon::Point(391, 374)) << std::endl;
    std::cout << "pt (0,0) is inside: " << polygon.CheckInside(Polygon::Point(0, 0)) << std::endl;

    return 0;
}