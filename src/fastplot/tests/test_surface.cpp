
#include <cmath>

// #include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkFloatArray.h>
#include <vtkStructuredGrid.h>
#include <vtkMath.h>
#include <vtkPointData.h>
#include <vtkDataSetMapper.h>
#include <vtkPolyDataMapper.h>
#include <vtkStructuredGridGeometryFilter.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkShrinkFilter.h>
#include <vtkDoubleArray.h>
#include <vtkStructuredGridOutlineFilter.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>

int main(int, char *[])
{
    // Create a grid
    vtkSmartPointer<vtkStructuredGrid> grid =
        vtkSmartPointer<vtkStructuredGrid>::New();

    int size_x = 300;
    int size_y = 400;
    grid->SetDimensions(size_x, size_y, 1);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->SetNumberOfPoints(size_x * size_y);
    for (int j = 0; j < size_y; j++)
        for (int i = 0; i < size_x; i++)
            points->InsertNextPoint(i, j, (std::sin(i) + std::cos(j)));
    grid->SetPoints(points);

    vtkSmartPointer<vtkFloatArray> colors = vtkSmartPointer<vtkFloatArray>::New();
    colors->SetNumberOfComponents(1);
    colors->SetNumberOfTuples(size_x * size_y);
    int k = 0;
    for (int j = 0; j < size_y; j++)
        for (int i = 0; i < size_x; i++)
        {
            colors->InsertComponent(k, 0, (std::sin(i) + std::cos(j)));
            k++;
        }
    grid->GetPointData()->SetScalars(colors);

    vtkSmartPointer<vtkStructuredGridGeometryFilter> geometryFilter =
        vtkSmartPointer<vtkStructuredGridGeometryFilter>::New();
    geometryFilter->SetInputData(grid);
    geometryFilter->Update();

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(geometryFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor =
        vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    vtkSmartPointer<vtkStructuredGridOutlineFilter> outlineFilter =
        vtkSmartPointer<vtkStructuredGridOutlineFilter>::New();
    outlineFilter->SetInputData(grid);
    outlineFilter->Update();
    vtkSmartPointer<vtkPolyDataMapper> outlineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    outlineMapper->SetInputConnection(outlineFilter->GetOutputPort());
    vtkSmartPointer<vtkActor> outline = vtkSmartPointer<vtkActor>::New();
    outline->SetMapper(outlineMapper);
    outline->GetProperty()->SetColor(0, 0, 0);

    // Visualize
    vtkSmartPointer<vtkRenderer> renderer =
        vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
        vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
        vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    renderer->AddActor(actor);
    renderer->AddActor(outline);
    renderer->SetBackground(.2, .3, .4);

    renderWindow->Render();
    renderWindowInteractor->Start();

    return EXIT_SUCCESS;
}