#include <iostream>

#include "lightview/lightviewer.hpp"

using namespace librav;

int main()
{
    LightViewer viewer;

    viewer.SetupViewer();

    viewer.Start();

    return 0;
}