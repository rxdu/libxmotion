/* 
 * collision.cpp
 * 
 * Created on: Aug 09, 2018 01:41
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "collision/collision.hpp"

#define CUTE_C2_IMPLEMENTATION
#include "collision/cute_c2.h"

using namespace librav;

bool Collision::Check(Polygon polygon1, Polygon polygon2)
{
    for (auto &part1 : polygon1.convex_partitions_)
    {
        for (auto &part2 : polygon2.convex_partitions_)
        {
            c2Poly p1, p2;

            // copy data to p1
            p1.count = part1.GetPointNumer();
            int32_t vcount = 0;
            for (auto it = part1.point_begin(); it != part1.point_end(); ++it)
            {
                p1.verts[vcount].x = (*it).x();
                p1.verts[vcount].y = (*it).y();
                ++vcount;
            }

            // copy data to p2
            p2.count = part2.GetPointNumer();
            vcount = 0;
            for (auto it = part2.point_begin(); it != part2.point_end(); ++it)
            {
                p2.verts[vcount].x = (*it).x();
                p2.verts[vcount].y = (*it).y();
                ++vcount;
            }

            // make c2 polygon
            c2MakePoly(&p1);
            c2MakePoly(&p2);

            // check collision
            if (c2PolytoPoly(&p1, nullptr, &p2, nullptr))
                return true;
        }
    }
    return false;
}
