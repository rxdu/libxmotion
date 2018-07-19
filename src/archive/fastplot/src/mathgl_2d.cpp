/* 
 * mathgl_2d.cpp
 * 
 * Created on: Mar 14, 2018 23:17
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "fastplot/mathgl_2d.hpp"

using namespace librav;

int MathGLDens::Draw(mglGraph *gr)
{
    SetBasicElements(gr, "Dens plot", false, 0, 0);
    ApplyRangeSettings(gr);
    SetAxisLabels(gr);
    
    gr->Box("", false);
    gr->Axis();
    gr->Dens(z_data_);

    return 0;
}

int MathGLSurf::Draw(mglGraph *gr)
{
    SetBasicElements(gr, "Surf plot", true);
    ApplyRangeSettings(gr);
    SetAxisLabels(gr);

    gr->Box("", false);
    gr->Axis();
    gr->Surf(z_data_);

    return 0;
}

int MathGLMesh::Draw(mglGraph *gr)
{
    SetBasicElements(gr, "Mesh plot", true);
    ApplyRangeSettings(gr);
    SetAxisLabels(gr);

    gr->Box("", false);
    gr->Axis();
    gr->Mesh(z_data_);

    return 0;
}