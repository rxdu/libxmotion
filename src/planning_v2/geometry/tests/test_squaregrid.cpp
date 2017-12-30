#include <iostream>

#include "geometry/square_grid.hpp"

using namespace librav;

int main()
{
    SquareGrid grid(3,3);

    // grid.SetCellOccupancy(1,1, OccupancyType::OCCUPIED);
    // grid.SetCellOccupancy(2,2, OccupancyType::UNKONWN);

    for(int32_t y = 0; y < grid.row_size_; y++)
        for(int32_t x = 0; x < grid.col_size_; x++)
            grid.grid_cells_[x][y]->PrintInfo();

    // check assert()
    //grid.GetIDFromIndex(3,1);
    //grid.SetCellOccupancy(3,1, OccupancyType::OCCUPIED);
    for(int32_t y = 0; y < grid.row_size_; y++)
        for(int32_t x = 0; x < grid.col_size_; x++)
        {
            int64_t id = grid.GetIDFromCoordinate(x,y);
            auto neighbours = grid.GetNeighbours(x,y, true);
            std::cout << "id: " << id << std::endl;
            for(auto& neighbour : neighbours)
                std::cout << " - neighbour: " << neighbour->id_ << std::endl;
        }
    

    return 0;
}