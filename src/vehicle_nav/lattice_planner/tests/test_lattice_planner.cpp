#include <memory>
#include <iostream>
#include <cstdint>

#include "road_map/road_map.hpp"

#include "lattice_planner/lattice_planner.hpp"

#include "lightviz/lightviz.hpp"

using namespace librav;

class GaussianPositionThreat
{
  public:
    GaussianPositionThreat(double pos_x, double pos_y, double sigma1, double sigma2) : pos_x_(pos_x),
                                                                                       pos_y_(pos_y),
                                                                                       sigma_1_(sigma1),
                                                                                       sigma_2_(sigma2)
    {
        // update coeff1_ and coeff2_
        SetParameters(pos_x, pos_y, sigma1, sigma2);
    }

    double GetCenterPositionX() const { return pos_x_; }
    double GetCenterPositionY() const { return pos_y_; }

    void SetParameters(double pos_x, double pos_y, double sigma1, double sigma2)
    {
        pos_x_ = pos_x;
        pos_y_ = pos_y;
        sigma_1_ = sigma1;
        sigma_2_ = sigma2;

        coeff1_ = (2 * M_PI * sigma_1_ * sigma_2_);
        coeff2_ = -(2 * sigma_1_ * sigma_1_);
        coeff3_ = -(2 * sigma_2_ * sigma_2_);
    }
    double operator()(double x, double y)
    {
        double x_err = x - pos_x_;
        double y_err = y - pos_y_;

        double val = std::exp(x_err * x_err / coeff2_ + y_err * y_err / coeff3_) / coeff1_;

        return val;
    }

  private:
    double pos_x_ = 0;
    double pos_y_ = 0;
    double sigma_1_ = 1;
    double sigma_2_ = 1;

    double coeff1_ = 0;
    double coeff2_ = 1;
    double coeff3_ = 1;
};

int main()
{
    /********** create road map **********/
    std::shared_ptr<RoadMap> map = std::make_shared<RoadMap>("/home/rdu/Workspace/librav/data/road_map/intersection_single_lane_with_centerline.osm");

    if (!map->MapReady())
    {
        std::cout << "map didn't load correctly" << std::endl;
        return -1;
    }

    map->PrintInfo();

    std::shared_ptr<LatticeManager> lm = std::make_shared<LatticeManager>();
    lm->LoadPrimitivesFromFile("/home/rdu/mp.1s20mph-3.data");

    LatticePlanner planner(lm, map);

    auto ego_route = map->FindShortestRouteName("s4", "s1");
    planner.SetEgoPlanningRoute(ego_route);

    Polygon fp;
    // fp.AddPoint(0.6, 1.0);
    // fp.AddPoint(-0.6, 1.0);
    // fp.AddPoint(-0.6, -1.0);
    // fp.AddPoint(0.6, -1.0);
    // this set works
    // fp.AddPoint(0.5, 1.0);
    // fp.AddPoint(-0.5, 1.0);
    // fp.AddPoint(-0.5, -1.0);
    // fp.AddPoint(0.5, -1.0);
    // this set works
    // fp.AddPoint(0.55, 1.2);
    // fp.AddPoint(-0.55, 1.2);
    // fp.AddPoint(-0.55, -1.2);
    // fp.AddPoint(0.55, -1.2);
    fp.AddPoint(1.2 * 2, 0.9);
    fp.AddPoint(1.2 * 2, -0.9);
    fp.AddPoint(-1.2 * 2, -0.9);
    fp.AddPoint(-1.2 * 2, 0.9);
    planner.SetVehicleFootprint(fp);

    // LatticeNode start(0, 0, 0);
    // LatticeNode goal(20, 20, -M_PI / 6.0);
    // LatticeNode start(57, 36, 80.0 / 180.0 * M_PI);
    // LatticeNode goal(20, 66.5, M_PI);
    LatticeNode start(57, 36, 85.0 / 180.0 * M_PI);
    // LatticeNode goal(23, 65.8, 180.0 / 180.0 * M_PI);
    // LatticeNode goal(22, 66.8, 180.0 / 180.0 * M_PI);
    LatticeNode goal(4, 68, 180.0 / 170.0 * M_PI);

    // auto path = planner.Search(start, 5);
    auto path = planner.AStarSearch(start, goal, 8, 10);
    // auto path = planner.BFSSearch(start, goal, 8, 10);

    // lm->SavePrimitivesToFile(path, "path");

    // if (!path.empty())
    // {
    //     std::vector<GridCoordinate> waypoints;
    //     for (auto &mp : path)
    //     {
    //         for (auto &nd : mp.nodes)
    //         {
    //             auto grid_pos = map->coordinate_.ConvertToGridPixel(CartCooridnate(nd.x, nd.y));

    //             waypoints.push_back(GridCoordinate(grid_pos.x, grid_pos.y));
    //         }
    //     }
    //     LightViz::ShowPathOnMatrixAsColorMap(drivable_mask->GetGridMatrix(true), waypoints, "lplanner", true);
    // }

    // LightViz::ShowLanePolylines(map->GetAllLaneBoundPolylines(), map->GetAllLaneCenterPolylines());
    auto path_line = planner.ConvertPathToPolyline(path);
    // LightViz::ShowPathInLane(map->GetAllLaneBoundPolylines(), map->GetAllLaneCenterPolylines(), path_line);

    GaussianPositionThreat threat_model(60, 60, 2, 2);
    LightViz::ShowPathLaneInField(map->GetAllLaneBoundPolylines(), map->GetAllLaneCenterPolylines(), path_line,
                                  threat_model.GetCenterPositionX(), threat_model.GetCenterPositionY(), threat_model);

    return 0;
}