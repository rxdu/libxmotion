#include <iostream>
#include "collision/collision.hpp"

using namespace librav;

int main()
{
    Polygon p1, p2;

    p1.AddPoint(5, 2);
    p1.AddPoint(5, 6);
    p1.AddPoint(1, 6);
    p1.AddPoint(1, 2);

    p2.AddPoint(7, 5);
    p2.AddPoint(7, 8);
    p2.AddPoint(4, 8);
    p2.AddPoint(4, 5);

    // p2.AddPoint(7, 5);
    // p2.AddPoint(7, 8);
    // p2.AddPoint(4, 8);
    // p2.AddPoint(4, 7);
    // p2.AddPoint(6, 7);
    // p2.AddPoint(6, 5);

    // p2.AddPoint(7, 5);
    // p2.AddPoint(7, 8);
    // p2.AddPoint(4, 8);
    // p2.AddPoint(4, 6);
    // p2.AddPoint(5, 6);
    // p2.AddPoint(5, 5);

    p1.ConvexDecomposition();
    p2.ConvexDecomposition();

    // Polygon polygon;
    // polygon.AddPoint(391, 374);
    // polygon.AddPoint(240, 431);
    // polygon.AddPoint(252, 340);
    // polygon.AddPoint(374, 320);
    // polygon.AddPoint(289, 214);
    // polygon.AddPoint(134, 390);
    // polygon.AddPoint(68, 186);
    // polygon.AddPoint(154, 259);
    // polygon.AddPoint(161, 107);
    // polygon.AddPoint(435, 108);
    // polygon.AddPoint(208, 148);
    // polygon.AddPoint(295, 160);
    // polygon.AddPoint(421, 212);
    // polygon.AddPoint(441, 303);

    // Polygon polygon2 = polygon;

    // polygon.ConvexDecomposition();
    // polygon2.ConvexDecomposition();

    std::cout << "collision between p1 and p2: " << Collision::Check(p1, p2) << std::endl;
    std::cout << "collision between p1 and p2: " << p1.Intersect(p2) << std::endl;

    // std::cout << "collision between p1 and p2: " << Collision::Check(polygon, polygon2) << std::endl;

    return 0;
}