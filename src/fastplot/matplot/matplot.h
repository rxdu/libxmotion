#ifndef __MATPLOT_VTK
#define __MATPLOT_VTK

/*
 * MatPlot_VTK
 *
 * Simple plotting of vectors and matrices based on
 * the VTK libraries. 
 *
 * Four classes:
 * Plot2D_VTK  - 2d plotting akin to plot(x,y)
 * Surf_VTK    - Surface plotting akin to surf(x,y,z)
 * Contour_VTK - Contour plotting
 * Quiver_VTK  - Vector field plot
 *
 * See examples.cpp for usage instructions.
 *
 * These classes are released "as is", without any implied 
 * warranty or fitness for any particular purpose.  
 *
 * Dag Lindbo, dag@csc.kth.se, 2007-12-09
 */

// matrix size functions
// #include "ublas_dims.h"
#include "mtl4_dims.h"

// system includes
#include <assert.h>
#include <string>

// vtk includes used by templates
#include "vtkFloatArray.h"
#include "vtkPointData.h"
#include "vtkPoints.h"
#include "vtkRectilinearGrid.h"
#include "vtkRenderer.h"
#include "vtkStructuredGrid.h"
#include "vtkXYPlotActor.h"

namespace matplot
{

    void render_interactive(vtkRenderer *rend, int xPix, int yPix);

    void render_interactive_cam(vtkRenderer *rend, int xPix, int yPix,
				double cam[3], double focal[3]);

    void render_to_png(vtkRenderer *rend, int xPix, int yPix,std::string fname);

    void render_to_png_cam(vtkRenderer *rend, int xPix, int yPix, 
			   std::string fname, double cam[3], double focal[3]);

    /** Plot vectors x versus y. 
     * Inspired by "plot" in Matlab. 
     * 
     * Notes:
     * 
     * Call plot(x,y,<...>) multiple times before show() to 
     * get multiple curves in one figure.
     *
     * \author Dag Lindbo
     */
    class Plot2D_VTK
    {
    public:

	Plot2D_VTK(std::string x_label="x", std::string y_label="y", 
		   int xpix = 800, int ypix = 600);
	~Plot2D_VTK();

	/** Insert curve x vs.\ y into plot. 
	 * - Default color is blue, {0 , 0, 1}.
	 * - Default line style is solid line, "-".
	 */
	template<typename Vec_t>
	void plot(const Vec_t &x, const Vec_t &y)
	{
	    double color[3] = { 0, 0, 1.0 };
	    plot(x, y, color, "-");
	}

	/** Insert curve x vs.\ y into plot (specify color and line style). 
	 * Color specification: 
	 * 		- RGB (normalized decimal), {0.32, 0.1, 0.0}
	 * 
	 * Line style specification:
	 * 		-	"-" 			solid line
	 * 		-	"." 			dotted line
	 * 		-	".-" or "-." 	solid line with dots
	 */
	template<typename Vec_t>
	void plot(const Vec_t &x, const Vec_t &y, const double col[3],
		  const std::string linespec)
	{
	    int i, N = vec_dim(x), plot_points = 0, plot_lines = 0;

	    vtkRectilinearGrid *curve;
	    vtkFloatArray *yVal;
	    vtkFloatArray *xVal;

	    // determine line style
	    if (linespec == "-")
		plot_lines = 1;
	    else if (linespec == ".")
		plot_points = 1;
	    else if (linespec == ".-" || linespec == "-.")
	    {
		plot_points = 1;
		plot_lines = 1;
	    }

	    // put (x,y) into VTK FloatArrays
	    xVal = vtkFloatArray::New();
	    yVal = vtkFloatArray::New();

	    for (i=0; i<N; i++)
	    {
		xVal->InsertNextTuple1(x[i]);
		yVal->InsertNextTuple1(y[i]);
	    }

	    // Make a VTK Rectlinear grid from arrays
	    curve = vtkRectilinearGrid::New();
	    curve->SetDimensions(N, 1, 1);
	    curve->SetXCoordinates(xVal);
	    curve->GetPointData()->SetScalars(yVal);

	    // attach gridfunction to plot
	    xyplot->AddInput(curve);

	    // how to read data
	    xyplot->SetXValuesToValue();

	    // set attributes
	    xyplot->SetPlotColor(plot_no, col[0], col[1], col[2]);
	    xyplot->SetPlotLines(plot_no, plot_lines);
	    xyplot->SetPlotPoints(plot_no, plot_points);

	    plot_no++;

	    xVal->Delete();
	    yVal->Delete();
	    curve->Delete();
	}

	void show();
	void draw_to_png(std::string filename);

    private:
	int xPix;
	int yPix;

	int plot_no;

	vtkRenderer* rend;
	vtkXYPlotActor* xyplot;
    };

    /** Plot z = f(x,y) surface.
     * Inspired by "surf" in Matlab
     * 
     * \author Dag Lindbo
     */
    class Surf_VTK
    {
    public:

	Surf_VTK(int px = 800, int py = 600);
	~Surf_VTK();

	/** Create surface plot.
	 * Matrix z, with respect to vectors x and y.
	 */
	template<typename Vec_t, typename Mat_t>
	void surf(const Vec_t &x, const Vec_t &y, const Mat_t &z)
	{
	    geometry(x, y, z);
	    renderer(false, true, true, false);
	    render_interactive(rend, xPix, yPix);
	}

	/** Create surface plot.
	 * Matrix z, with respect to vectors x and y. 
	 * 
	 * Warp z-axis to produce better fit:
	 * - do_warp = true
	 */
	template<typename Vec_t, typename Mat_t>
	void surf(const Vec_t &x, const Vec_t &y, const Mat_t &z, bool do_warp)
	{
	    geometry(x, y, z);
	    renderer(false, true, true, do_warp);
	    render_interactive(rend, xPix, yPix);
	}
    
	/** Create surface plot.
	 * Matrix z, with respect to vectors x and y. 
	 * 
	 * Warp z-axis to produce better fit:
	 * - do_warp = true
	 * 
	 * Camera control:
	 * - specify camera position: cam = { -15.0, 10.0, 12.0 }
	 * - specify focal point: focal = { 0, 0, 0 }
	 */
	template<typename Vec_t, typename Mat_t>
	void surf(const Vec_t &x, const Vec_t &y, const Mat_t &z, bool do_warp,
		  double observer[3], double focal[3])
	{
	    geometry(x, y, z);
	    renderer(false, true, true, do_warp);
	    render_interactive_cam(rend, xPix, yPix, observer, focal);
	}

	/** Create surface plot and render to file
	 * Matrix z, with respect to vectors x and y. 
	 * 
	 * Warp z-axis to produce better fit:
	 * - do_warp = true
	 * 
	 * Camera control:
	 * - specify camera position: cam = { -15.0, 10.0, 12.0 }
	 * - specify focal point: focal = { 0, 0, 0 }
	 */
	template<typename Vec_t, typename Mat_t>
	void surf_to_file(const Vec_t &x, const Vec_t &y, const Mat_t &z,
			  bool do_warp, std::string fname, double observer[3], 
			  double focal[3])
	{
	    geometry(x, y, z);
	    renderer(false, true, true, do_warp);
	    render_to_png_cam(rend, xPix, yPix, fname, observer, focal);
	}

	void purge();

    private:
	vtkStructuredGrid *gridfunc;
	vtkRenderer *rend;

	double Lxy, Lz;
	bool has_data;

	int xPix, yPix;

	template<typename Vec_t, typename Mat_t>
	void geometry(const Vec_t &x, const Vec_t &y, const Mat_t &z)
	{
	    const unsigned int Nx = vec_dim(x);
	    const unsigned int Ny = vec_dim(y);
	    unsigned int i, j, k;

	    // make sure the input is ok and that this surfaceplot is free
	    assert(Nx == mat_dim(z,1));
	    assert(Ny == mat_dim(z,2));
	    assert(!has_data);

	    // determine x-y range of data
	    if (x(Nx-1)-x(0) > y(Ny-1)-y(0))
		Lxy = x(Nx-1)-x(0);
	    else
		Lxy = y(Ny-1)-y(0);
	    double z_low = 10000, z_upp = -10000;

	    // put data, z, into a 2D structured grid
	    gridfunc->SetDimensions(Nx, Ny, 1);

	    vtkPoints *points = vtkPoints::New();
	    for (j = 0; j < Ny; j++)
	    {
		for (i = 0; i < Nx; i++)
		{
		    points->InsertNextPoint(x(i), y(j), z(i, j));

		    if (z(i, j)< z_low)
			z_low = z(i, j);
		    if (z(i, j)> z_upp)
			z_upp = z(i, j);
		}
	    }
	    gridfunc->SetPoints(points);

	    // get scalar field from z-values
	    vtkFloatArray *colors = vtkFloatArray::New();
	    colors->SetNumberOfComponents(1);
	    colors->SetNumberOfTuples(Nx*Ny);
	    k = 0;
	    for (j = 0; j < Ny; j++)
		for (i = 0; i < Nx; i++)
		{
		    colors->InsertComponent(k, 0, z(i, j));
		    k++;
		}

	    gridfunc->GetPointData()->SetScalars(colors);

	    points->Delete();
	    colors->Delete();

	    has_data = true;
	    Lz = z_upp-z_low;
	}

	void renderer(bool, bool, bool, bool);

    };

    /** Plot contour lines for a function in the plane.
     * Inspired by "contour" in Matlab
     * 
     * \author Dag Lindbo
     */
    class Contour_VTK
    {
    public:

	Contour_VTK(int px = 800, int py = 600);
	~Contour_VTK();

	/** Create contour plot.
	 * Matrix z vs. vectors x and y. 
	 * Produces a default number of contour lines (10) and
	 * colors the lines instead of the underlying surface.
	 */
	template<typename Vec_t, typename Mat_t>
	void contour(const Vec_t &x, const Vec_t &y, const Mat_t &z)
	{
	    geometry(x, y, z);
	    renderer(true, false, 10);
	    render_interactive(rend, xPix, yPix);
	}

	/**Create contour plot.
	 * Matrix z vs. vectors x and y.
	 * 
	 * Number of contour lines:
	 * - num_lines
	 * 
	 * Coloring:
	 * - draw_surf = false: color contour lines and omits underlying surface
	 * - draw_surf = true:  draw contour lines white and color the 
	 *                      underlying surface 
	 */
	template<typename Vec_t, typename Mat_t>
	void contour(const Vec_t &x, const Vec_t &y, const Mat_t &z, 
		     bool draw_surf, int num_lines)
	{
	    geometry(x, y, z);
	    renderer(true, draw_surf, num_lines);
	    render_interactive(rend, xPix, yPix);
	}

	/**Create contour plot and render to file.
	 * Matrix z vs. vectors x and y.
	 * 
	 * Number of contour lines:
	 * - num_lines
	 * 
	 * Coloring:
	 * - draw_surf = false: color contour lines and omits underlying surface
	 * - draw_surf = true:  draw contour lines white and color the 
	 *                      underlying surface 
	 */
	template<typename Vec_t, typename Mat_t>
	void contour_to_file(const Vec_t &x, const Vec_t &y, const Mat_t &z,
			     bool draw_surf, int num_lines, std::string fname)
	{
	    geometry(x, y, z);
	    renderer(true, draw_surf, num_lines);
	    render_to_png(rend, xPix, yPix, fname);
	}

	void purge();

    private:
	vtkRectilinearGrid *gridfunc;
	vtkRenderer *rend;

	bool has_data;

	int xPix, yPix;

	template<typename Vec_t, typename Mat_t>
	void geometry(const Vec_t &x, const Vec_t &y, const Mat_t &z)
	{
	    const unsigned int Nx = vec_dim(x);
	    const unsigned int Ny = vec_dim(y);
	    unsigned int i, j, k;

	    // make sure the input is ok and that this contourplot is free
	    assert(Nx == mat_dim(z,1));
	    assert(Ny == mat_dim(z,2));
	    assert(!has_data);

	    // x and y vectors go into vtkFloatArray 
	    vtkFloatArray *xcoord = vtkFloatArray::New();
	    xcoord->SetNumberOfComponents(1);
	    xcoord->SetNumberOfTuples(Nx);
	    vtkFloatArray *ycoord = vtkFloatArray::New();
	    ycoord->SetNumberOfComponents(1);
	    ycoord->SetNumberOfTuples(Ny);

	    for (i=0; i<Nx; i++)
		xcoord->InsertComponent(i, 0, x(i));
	    for (i=0; i<Ny; i++)
		ycoord->InsertComponent(i, 0, y(i));

	    // Create rectilinear grid
	    gridfunc->SetDimensions(Nx, Ny, 1);
	    gridfunc->SetXCoordinates(xcoord);
	    gridfunc->SetYCoordinates(ycoord);

	    // add z-values as scalars to grid
	    vtkFloatArray *colors = vtkFloatArray::New();
	    colors->SetNumberOfComponents(1);
	    colors->SetNumberOfTuples(Nx*Ny);
	    k = 0;
	    for (j = 0; j < Ny; j++)
		for (i = 0; i < Nx; i++)
		{
		    colors->InsertComponent(k, 0, z(i, j));
		    k++;
		}

	    gridfunc->GetPointData()->SetScalars(colors);

	    colors->Delete();
	    xcoord->Delete();
	    ycoord->Delete();

	    has_data = true;
	}
    
	void renderer(bool, bool, int);

    };

    /** Plot vector-valued function in the plane.
     * Inspired by "quiver" in Matlab
     * 
     * \author Dag Lindbo
     */
    class Quiver_VTK
    {
    public:

	Quiver_VTK(int px = 800, int py = 600);
	~Quiver_VTK();

	/** Create vector arrow plot (quiver).
	 * Pointwise vecotrs in matrices u and v, at grid
	 * points given by vectors x and y. Color by magnitude.
	 * Defaults to no scaling of arrow lengths. 
	 */
	template<typename Vec_t, typename Mat_t>
	void quiver(const Vec_t &x, const Vec_t &y, const Mat_t &u,
		    const Mat_t &v)
	{
	    geometry(x, y, u, v);
	    renderer(1.0);
	    render_interactive(rend, xPix, yPix);
	}

	/** Create vector arrow plot (quiver).
	 * Pointwise vectors in matrices u and v, at grid
	 * points given by vectors x and y. Color by magnitude.
	 * 
	 * Scales arrows by a factor s. 
	 */
	template<typename Vec_t, typename Mat_t>
	void quiver(const Vec_t &x, const Vec_t &y, const Mat_t &u,
		    const Mat_t &v, double s)
	{
	    geometry(x, y, u, v);
	    renderer(s);
	    render_interactive(rend, xPix, yPix);
	}

	/** Create vector arrow plot (quiver) and render to file.
	 * Pointwise vectors in matrices u and v, at grid
	 * points given by vectors x and y. Color by magnitude
	 * 
	 * Scales arrows by a factor s. 
	 */
	template<typename Vec_t, typename Mat_t>
	void quiver_to_file(const Vec_t &x, const Vec_t &y, const Mat_t &u,
			    const Mat_t &v, double s, std::string filename)
	{
	    geometry(x, y, u, v);
	    renderer(s);
	    render_to_png(rend, xPix, yPix, filename);
	}

	void purge();

    private:
	vtkRectilinearGrid *gridfunc;
	vtkRenderer *rend;

	bool has_data;

	int xPix, yPix;

	template<typename Vec_t, typename Mat_t>
	void geometry(const Vec_t& x, const Vec_t& y, const Mat_t& u,
		      const Mat_t& v)
	{
	    const unsigned int Nx = vec_dim(x);
	    const unsigned int Ny = vec_dim(y);
	    unsigned int i, j, k;

	    // make sure the input is ok and that this contourplot is free
	    assert(Nx == mat_dim(u,1));
	    assert(Ny == mat_dim(u,2));
	    assert(Nx == mat_dim(v,1));
	    assert(Ny == mat_dim(v,2));
	    assert(!has_data);

	    // x and y vectors go into vtkFloatArray 
	    vtkFloatArray *xcoord = vtkFloatArray::New();
	    xcoord->SetNumberOfComponents(1);
	    xcoord->SetNumberOfTuples(Nx);
	    vtkFloatArray *ycoord = vtkFloatArray::New();
	    ycoord->SetNumberOfComponents(1);
	    ycoord->SetNumberOfTuples(Ny);

	    for (i=0; i<Nx; i++)
		xcoord->InsertComponent(i, 0, x(i));
	    for (i=0; i<Ny; i++)
		ycoord->InsertComponent(i, 0, y(i));

	    // Create rectilinear grid
	    gridfunc->SetDimensions(Nx, Ny, 1);
	    gridfunc->SetXCoordinates(xcoord);
	    gridfunc->SetYCoordinates(ycoord);

	    // add magnitude of (u,v) as scalars to grid
	    vtkFloatArray *colors = vtkFloatArray::New();
	    colors->SetNumberOfComponents(1);
	    colors->SetNumberOfTuples(Nx*Ny);

	    // add vector (u,v) to grid
	    vtkFloatArray *vectors = vtkFloatArray::New();
	    vectors->SetNumberOfComponents(3);
	    vectors->SetNumberOfTuples(Nx*Ny);

	    k = 0;
	    for (j = 0; j < Ny; j++)
		for (i = 0; i < Nx; i++)
		{
		    colors->InsertTuple1(k,sqrt(u(i,j)*u(i,j)+v(i,j)*v(i,j)));
		    vectors->InsertTuple3(k, u(i, j), v(i, j), 0.0);
		    k++;
		}

	    gridfunc->GetPointData()->SetScalars(colors);
	    gridfunc->GetPointData()->SetVectors(vectors);

	    vectors->Delete();
	    colors->Delete();
	    xcoord->Delete();
	    ycoord->Delete();

	    has_data = true;
	}

	void renderer(double);

    };

}

#endif
