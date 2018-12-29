#include <iostream>
#include <cstdint>

#include "sampling/details/space/realvector_space.hpp"

using namespace librav;

int main()
{
    RealVectorSpace<3> rvspace;

    rvspace.SetBound(0, 1, 1.5);
    rvspace.SetBound(1, 2, 3.5);
    rvspace.SetBound(2, 3, 4.5);

    rvspace.PrintInfo();

    RealVectorSpace<2> rvspace2({{1, 1.5}, {1, 1.5}});
    rvspace2.PrintInfo();

    return 0;
}