#include <iostream>

#define CUTE_C2_IMPLEMENTATION
#include "collision/cute_c2.h"

int main()
{
    c2v pt[5];

    pt[0].x = 0;
    pt[0].y = 0;
    pt[1].x = 0;
    pt[1].y = 2;
    pt[2].x = 2;
    pt[2].y = 0;
    pt[3].x = 2;
    pt[3].y = 2;
    pt[4].x = 1;
    pt[4].y = 1;

    c2Poly poly1,poly2;

    const int pt_num1 = 4;
    poly1.count = pt_num1;
    for (int i = 0; i < pt_num1; ++i)
        poly1.verts[i] = pt[i];
    c2MakePoly(&poly1);

    const int pt_num2 = 3;
    poly2.count = pt_num2;
    for (int i = 0; i < pt_num2; ++i)
        poly2.verts[i] = pt[i];
    c2MakePoly(&poly2);

    std::cout << "collision: " << c2PolytoPoly(&poly1, nullptr, &poly2, nullptr) << std::endl;

    return 0;
}