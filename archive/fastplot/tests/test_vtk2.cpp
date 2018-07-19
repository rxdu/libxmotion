#include <cstdint>

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkIdList.h>
#include <vtkProperty.h>
#include <vtkStructuredGrid.h>
#include <vtkXMLStructuredGridWriter.h>
#include <vtkMath.h>
#include <vtkPointData.h>
#include <vtkDataSetMapper.h>
#include <vtkActor.h>
#include <vtkFloatArray.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkStructuredGridGeometryFilter.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLStructuredGridWriter.h>
#include <vtkStructuredGridOutlineFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkWarpScalar.h>

int main(int, char *[])
{
    // Create a grid
    vtkSmartPointer<vtkStructuredGrid> structuredGrid =
        vtkSmartPointer<vtkStructuredGrid>::New();

    vtkSmartPointer<vtkPoints> points =
        vtkSmartPointer<vtkPoints>::New();

    int32_t gridSize = 100;
    for (unsigned int j = 0; j < gridSize; j++)
    {
        for (unsigned int i = 0; i < gridSize; i++)
        {
            points->InsertNextPoint(i, j, 5*(std::sin(i/5.0) + std::cos(j/5.0))); // Make most of the points the same height
        }
    }

    // Specify the dimensions of the grid
    structuredGrid->SetDimensions(gridSize, gridSize, 1);
    structuredGrid->SetPoints(points);

    vtkSmartPointer<vtkFloatArray> colors = vtkSmartPointer<vtkFloatArray>::New();
    colors->SetNumberOfComponents(1);
    colors->SetNumberOfTuples(gridSize * gridSize);
    int32_t k = 0;
    for (int j = 0; j < gridSize; j++)
        for (int i = 0; i < gridSize; i++)
        {
            colors->InsertComponent(k, 0, (std::sin(i/5.0) + std::cos(j/5.0)));
            k++;
        }
    structuredGrid->GetPointData()->SetScalars(colors);

    vtkSmartPointer<vtkStructuredGridGeometryFilter> geometryFilter =
        vtkSmartPointer<vtkStructuredGridGeometryFilter>::New();
    geometryFilter->SetInputData(structuredGrid);
    geometryFilter->Update();

    // vtkSmartPointer<vtkWarpScalar> warp = vtkSmartPointer<vtkWarpScalar>::New();
	// double scale = 20;
	// warp->SetInputConnection(geometryFilter->GetOutputPort());
	// warp->XYPlaneOn();
	// warp->SetScaleFactor(scale);

    // Create a mapper and actor
    vtkSmartPointer<vtkDataSetMapper> gridMapper =
        vtkSmartPointer<vtkDataSetMapper>::New();
    // gridMapper->SetInputData(structuredGrid);
    gridMapper->SetInputConnection(geometryFilter->GetOutputPort());
    // gridMapper->SetInputConnection(warp->GetOutputPort());

    vtkSmartPointer<vtkActor> gridActor =
        vtkSmartPointer<vtkActor>::New();
    gridActor->SetMapper(gridMapper);
    gridActor->GetProperty()->EdgeVisibilityOn();
    // gridActor->GetProperty()->SetEdgeColor(0, 0, 1);

    vtkSmartPointer<vtkStructuredGridOutlineFilter> outlineFilter =
        vtkSmartPointer<vtkStructuredGridOutlineFilter>::New();
    outlineFilter->SetInputData(structuredGrid);
    // outlineFilter->SetInputConnection(geometryFilter->GetOutputPort());
    // outlineFilter->SetInputConnection(warp->GetOutputPort());
    outlineFilter->Update();
    vtkSmartPointer<vtkPolyDataMapper> outlineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    outlineMapper->SetInputConnection(outlineFilter->GetOutputPort());
    vtkSmartPointer<vtkActor> outline = vtkSmartPointer<vtkActor>::New();
    outline->SetMapper(outlineMapper);
    outline->GetProperty()->SetColor(0, 0, 0);

    // Create a renderer, render window, and interactor
    vtkSmartPointer<vtkRenderer> renderer =
        vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
        vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
        vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Add the actor to the scene
    renderer->AddActor(gridActor);
    renderer->AddActor(outline);
    renderer->SetBackground(.2, .3, .4);

    // Render and interact
    renderWindow->Render();
    renderWindowInteractor->Start();

    return EXIT_SUCCESS;
}