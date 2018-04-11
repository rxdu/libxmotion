#include <iostream>

#include "road_network/road_map.hpp"

using namespace librav;

int main()
{
    std::vector<PolyLinePoint> bound_pts;
    bound_pts.emplace_back(1, 2);
    bound_pts.emplace_back(1, 4);
    bound_pts.emplace_back(3, 4);
    bound_pts.emplace_back(3, 2);
    bound_pts.emplace_back(1, 2);

    Polygon polygon(bound_pts);
    std::cout << "(2,2) is inside: " << polygon.InsidePolygon(PolyLinePoint(2, 2)) << std::endl;
    std::cout << "(0,2) is inside: " << polygon.InsidePolygon(PolyLinePoint(0, 2)) << std::endl;
    std::cout << "(2,5) is inside: " << polygon.InsidePolygon(PolyLinePoint(2, 5)) << std::endl;
    std::cout << "(2,3.5) is inside: " << polygon.InsidePolygon(PolyLinePoint(2, 3.5)) << std::endl;

    return 0;
}