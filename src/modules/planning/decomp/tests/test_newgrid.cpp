#include <iostream>

#include "decomp/square_grid.hpp"

using namespace ivnav;

int main()
{
    SquareGrid grid(5,5);

    for(int i = 0; i < 25; ++i)
        grid.GetCell(i)->PrintInfo();
    
    return 0;
}