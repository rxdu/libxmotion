#include <iostream>

#include "pose_viewer/pose_viewer.hpp"

using namespace ivnav;

int main()
{
    PoseViewer viewer;

    viewer.SetupViewer();
    viewer.Start();

    return 0;
}