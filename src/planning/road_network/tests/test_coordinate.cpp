#include <iostream>

#include "road_network/road_coordinate.hpp"
#include <liblanelet/LaneletPoint.hpp>

using namespace librav;
using namespace LLet;

int main()
{
    point_with_id_t origin = std::make_tuple(42.27252481191, -71.81207478181, -1);
    RoadCoordinateFrame coordinate;
    coordinate.SetOrigin(origin);

    // 42.27252381955 , - 71.81199431554
    // 42.27260221558 , - 71.8120734407

    auto cc0 = coordinate.ConvertToCartesian(GeoCoordinate(42.27252481191, -71.81207478181));
    auto cc1 = coordinate.ConvertToCartesian(GeoCoordinate(42.27252381955, -71.81199431554));    
    auto cc2 = coordinate.ConvertToCartesian(GeoCoordinate(42.27260221558, -71.8120734407));

    std::cout << "cc0 x: " << cc0.x << " , " << cc0.y << std::endl;
    std::cout << "cc1 x: " << cc1.x << " , " << cc1.y << std::endl;
    std::cout << "cc2 x: " << cc2.x << " , " << cc2.y << std::endl;

    return 0;
}