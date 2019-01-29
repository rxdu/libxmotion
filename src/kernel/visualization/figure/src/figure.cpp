//////////////////////////////////////////////////////////
// (c) Seminar of Applied Mathematics ETH 2016, D-MATH  //
// Author: Julien Gacon <jgacon@ethz.ch>                //
// Co-Author: Baranidharan Mohan                        //
//////////////////////////////////////////////////////////

// system includes
# include <iostream>
# include <utility> // pair
# include <vector>
# include <algorithm>
# include <numeric>
# include <functional>
# include <sstream> // needed for title layout
# include <limits>
# include <cstring> // needed for length of const char*
# include <memory>

// own includes
# include "figure/MglPlot.hpp"
# include "figure/MglLabel.hpp"
# include "figure/MglStyle.hpp"
# include "figure/figure.hpp"

namespace mgl {


/*******************************************************************
 *                    Universal functions                          *
 *******************************************************************/

/* prints mglData, used for debugging                       *
 * PRE : -                                                  *
 * POST: prints content of mglData argument to command line */
void print(const mglData& d)
{
  for (long i = 0; i < d.GetNx(); ++i){
    std::cout << d.a[i] << " ";
  }
  std::cout << "\n";
}


/* get minimal positive ( > 0 ) value of mglData                            *
 * PRE : -                                                                  *
 * POST: minimal positive value of argument,                                *
 *       std::numeric_limits<double>::max() if no positive value cotained,  *
 *       print a warning to std::cerr if a value <= 0 encountered           *
 * NOTE: this function is only used to check ranges in logarithmic scaling  * 
 *       therefore the warning                                              */
double minPositive(const mglData& d) {
  double result = std::numeric_limits<double>::max();
  bool print_warning = false;

  // iterate over the given data
  for (long i = 0; i < d.GetNx(); ++i){
    if (d.a[i] > 0){
      // if the data point is positive, check if it is smaller than the current minimum
      result = std::min(result, d.a[i]);
    }
    else {
      // if the data point is not positive it will not appear on the plot -> print warning
      print_warning = true;
    }
  }

  if (print_warning) {
    std::cerr << "* Figure - Warning * non-positive values of data will not appear on plot. \n";
  }

  return result;
}


/*******************************************************************
 *                   Figure class constructor                      *
 *******************************************************************/

/* constructor: set default style                                                                  *
 * PRE : pointer to mglGraph will not cease to exist until operations are performed on this Figure *
 * POST: default settings                                                                          */
Figure::Figure()
  : autoRanges_(true),
    axis_(true),
    barplot_(false),
    grid_(false),
    has_3d_(false),
    legend_(false),
    figHeight_(-1), // set to -1: later we will check if they have been changed manually, -1 means no
    figWidth_(-1),  //            any other value will mean that they've been changed
    plotHeight_(800), // quadratic plot, window size depends on wheter there are labels or not!
    plotWidth_(800),
    leftMargin_(-1),
    topMargin_(-1),
    fontSizePT_(4), // small font size
    gridCol_("{h7}"),
    gridType_("xy"),
    title_(""),
    xFunc_("x"),
    yFunc_("y"),
    zFunc_("z"),
    legendPos_(1,1),
    aspects_({1, 1, 1}), // normal axis, no shearing
    ranges_({ std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
          std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()}),
    zranges_({std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()}),
    styles_(MglStyle())
    {}

/*******************************************************************
 *                 Non-template member functions                   *
 *******************************************************************/

/* setting height of the plot                                    *
 * leftMargin                                                    *
 *  v                                                            * 
 * +----------+                                                  *
 * |          |  < topMargin                                     *  
 * | +------+ |                                                  *
 * | | plot | |                                                  *
 * | +------+ |                                                  * 
 * |          |                                                  *
 * +----------+                                                  *
 *    window                                                     *
 *                                                               *
 * PRE : height should be > 0                                    *
 * POST: height of the plot will later be set to given height    */
void Figure::setPlotHeight(const int height) {
  plotHeight_ = height;
}


/* setting width of the plot                                   *
 * PRE : width should be > 0                                   *
 * POST: width of the plot will later be set to given width    */
void Figure::setPlotWidth(const int width) {
  plotWidth_ = width;
}


/* setting top margin                                            *
 * PRE : > 0                                                     *
 * POST: top margin will be set to top                           */
void Figure::setTopMargin(const int top) {
  topMargin_ = top;
}


/* setting left margin                                           *
 * PRE : > 0                                                     *
 * POST: left margin will be set to left                         */
void Figure::setLeftMargin(const int left) {
  leftMargin_ = left;
}


/* setting height of the graphic                                 *
 * PRE : height should be > 0                                    *
 * POST: height of the graphic will later be set to given height */
void Figure::setHeight(const int height) {
  figHeight_ = height;
}


/* setting width of the graphic                                *
 * PRE : width should be > 0                                   *
 * POST: width of the graphic will later be set to given width */
void Figure::setWidth(const int width) {
  figWidth_ = width;
}


/* setting the font size                                          *
 * PRE : size should be > 0                                       *
 * POST: font size of the graphic will later be set to given size */
void Figure::setFontSize(const int size) {
  fontSizePT_ = size;
}


/* enable to manually add legend entries       *
 * PRE : -                                     *
 * POST: label + style are added to the legend */
void Figure::addlabel(const std::string& label, const std::string& style) {
  additionalLabels_.push_back(std::pair<std::string, std::string>(label, style));
}


/* change grid settings                                                         *
 * PRE : -                                                                      *
 * POST: No grid if on is false, grid style is gridType, grid color is gridCol. *
 *       For those arguments which are not given default settings are used.     */
void Figure::grid(bool on, const std::string& gridType,  const std::string& gridCol) {
  if (on){
    grid_ = true;
  }
  // although in the constructor grid_ is set to false this explicit setting to false
  // is necessary as otherwise the grid couldn't be turned off once activated
  else { 
    grid_ = false;
  }

  gridType_ = gridType;
  gridCol_ = gridCol;
}


/* setting x-axis label                         *
 * PRE : -                                      *
 * POST: xlabel initialized with given position */
void Figure::xlabel(const std::string& label, double pos) {
  xMglLabel_ = MglLabel(label, pos);
}


/* setting y-axis label                         *
 * PRE : -                                      *
 * POST: ylabel initialized with given position */
void Figure::ylabel(const std::string& label, double pos) {
  yMglLabel_ = MglLabel(label, pos);
}


/* set or unset legend                                       *
 * PRE : -                                                   *
 * POST: if on is true legend will be plotted, otherwise not */
void Figure::legend(const double& xPos, const double& yPos) {
  // print a warning if the user has given a position which will probably not appear on the plot
  if (std::abs(xPos) > 2 || std::abs(yPos) > 2) {
    std::cerr << "* Figure - Warning * Legend may be out of the graphic due to large xPos or yPos\n";
  }

  // set legend_ to true so we know we have to activate the legend later
  legend_ = true;
  // keeping track of the position of the legend
  legendPos_ = std::pair<double, double>(xPos, yPos);
}


/* plot a function given by a string                                                            *
 * PRE : proper format of the input, e.g.: "3*x^2 + exp(x)", see documentation for more details *
 * POST: plot the function in given style                                                       */
MglPlot& Figure::fplot(const std::string& function, std::string style) {
#if NDEBUG
  std::cout << "Called fplot!\n";
#endif

  // checking if a style is given, 
  // if yes: use it and delete it from the style-container,
  // if no: get a style from the style container
  if (style.size() == 0) {
    style = styles_.get_next();
  }
  else {
    styles_.eliminate(style);
  }

  // put the plot in the plot queue 
  plots_.emplace_back(std::unique_ptr<MglFPlot>(new MglFPlot(function, style)));
  return *plots_.back().get();
}


/* set ranges                                                   *
 * PRE : -                                                      *
 * POST: new ranges will be: x = [xMin, xMax], y = [yMin, yMax] */
void Figure::ranges(const double& xMin, const double& xMax, const double& yMin, const double& yMax) {
  // checking if the input is valid
  if (xMin > xMax || yMin > yMax){
    std::cerr << "In function Figure::ranges(): xMin must be smaller than xMax and yMin smaller than yMax!";
  }

  // ranges have been set manually, so disable automatic ranges setting
  autoRanges_ = false;

  // set ranges according to the input
  ranges_ = {xMin, xMax, yMin, yMax};
}


/* change ranges of plot                                                             *
 * PRE : -                                                                           *
 * POST: set ranges in such a way that all data is displayed                         *
 * NOTE: need the argument vertMargin to be able to set it to 0 when plotting in 3d  */
void Figure::setRanges(const mglData& xd, const mglData& yd, double vertMargin) {
#if NDEBUG
  std::cout << "setRanges for 2dim called\n";
#endif

  // initialize data
  double xMax(xd.Maximal()), yMax(yd.Maximal());
  double xMin(xd.Minimal()), yMin(yd.Minimal());

  // check for x and y axis if they are logarithmic,
  // if yes: if the minimal value is <= 0 set xMin (or yMin) to the smallest positive number in the data
  //         and if the maximal value is <= 0 no data will appear -> error message!
  // if no:  leave the old value
  if (xFunc_ == "lg(x)"){
    if (xMax <= 0){
      std::cerr << "In function Figure::setRanges() : Invalid ranges for logscaled plot - maximal x-value must be greater than 0.";
    }
    yMin = minPositive(xd);
  }
  if (yFunc_ == "lg(y)"){
    if (yMax <= 0){
      std::cerr << "In function Figure::setRanges() : Invalid ranges for logscaled plot - maximal y-value must be greater than 0.";
    }
    vertMargin = 0.; // no vertical margin in logscaling yet
    yMin = minPositive(yd);
  }

  // set new ranges
  const double yTot = yMax - yMin;
  ranges_[0] = std::min(xMin , ranges_[0]);
  ranges_[1] = std::max(xMax , ranges_[1]);
  ranges_[2] = std::min(yMin - yTot*vertMargin, ranges_[2]); // adding a slight margin in linear plots on bottom
  ranges_[3] = std::max(yMax + yTot*vertMargin, ranges_[3]); // .. and top
}


/* change ranges of the plotted region in 3d                        *
 * PRE : -                                                          *
 * POST: set the ranges in such a way that all data will be visible */
void Figure::setRanges(const mglData& xd, const mglData& yd, const mglData& zd) {
#if NDEBUG
  std::cout << "setRanges for 3dim called\n";
#endif
  // initializing data
  const double zMax(zd.Maximal());
  double zMin(zd.Minimal());

  // check if z-axis is logarithmic,
  // if yes: if the minimal value is <= 0 set zMin to the smallest positive number in the data
  //         and if the maximal value is <= 0 no data will appear -> error message!
  // if no:  leave the old value
  if (zFunc_ == "lg(z)"){
    if (zMax <= 0){
      std::cerr << "In function Figure::setRanges() : Invalid ranges for logscaled plot - maximal z-value must be greater than 0.";
    }
    zMin = minPositive(zd);
  }
  zranges_[0] = std::min(zranges_[0], zMin);
  zranges_[1] = std::max(zranges_[1], zMax);
  setRanges(xd, yd, 0.); // use this function to set the correct x and y ranges
}


/* (un-)set logscaling                                                               *
 * PRE : -                                                                           *
 * POST: linear, semilogx, semilogy or loglog scale according to bools logx and logy */
void Figure::setlog(bool logx, bool logy, bool logz) {
  // if logi is true set iFunc_ to "lg(i)", which will later be used to initialize the coordinate curvature (i=x,y,z)
  if (logx){
    xFunc_ = "lg(x)";
  }
  if (logy){
    yFunc_ = "lg(y)";
  }
  if (logz){
    zFunc_ = "lg(z)";
  }
}

/* setting title                                                    *
 * PRE : -                                                          *
 * POST: title_ variable set to 'text' with small font option (@)   */
void Figure::title(const std::string& text) {
  title_ = "@{" + text + "}";
}

/* save figure                                                              *
 * PRE : -                                                                  *
 * POST: write figure to 'file' in png-format if 'file' end on .png,        *
 *       and to eps-format otherwise                                        *
 * ! IMPORTANT NOTE !                                                       *
 * The methods on gr_ have to be called in a particular order:              *
 *  1. SetSize - first to be called as it deletes all content               *
 *  2. Set Ticks & Font (SetTuneTicks, SetTickLen, LoadFont, SetFontSizePT) *
 *              2d-plot                     3d-plot                         *
 *  3. SubPlot                           3. SetRanges                       *
 *  4. SetRanges                         4. Rotate                          *
 *  4. Title                             5. Title & Label                   *
 *  5. InPlot                            6. SetFunc                         *
 *  6. Label                             7. Grid                            *
 *  7. SetFunc                           8. Axis                            *
 *  8. Grid                              9. Box                             *
 *  9. Axis                              10. now do all kinds of plots      *
 *  10. Box                              11. AddLegend                      *
 *  11. now do all kinds of plots        12. Legend                         *
 *  12. AddLegend                        finally: WriteEPS/PNG              *
 *  13. Legend                                                              *
 *  finally: WriteEPS/PNG                                                   *
 * If this order is violated the layout may change drastically!             */
void Figure::save(const std::string& file) {
  mglGraph gr_; // graph in which the plots will be saved

  // check if the plot, fig and top/left margins havent been set manually
  if (figWidth_ == -1 || figHeight_ == -1 || topMargin_ == -1 || leftMargin_ == -1) {
    // means there is a label
    if (yMglLabel_.str_.size() != 0 || xMglLabel_.str_.size() != 0) {
      figWidth_ = plotWidth_ + 300;
      figHeight_ = plotHeight_ + 270;
      topMargin_ = 100; // leave sufficient space for the labels
      leftMargin_ = 150;
    }
    else {
      figWidth_ = plotWidth_ + 200;
      figHeight_ = plotHeight_ + 200;
      topMargin_ = 100; // just a small margin, space for axis ticks
      leftMargin_ = 100;
    }
  }

  // Set size. This *must* be the first function called on the mglGraph
  gr_.SetSize(figWidth_, figHeight_);

  // Set position of scale annotations
  gr_.SetTuneTicks(true, 1.04);
  // Shorten tick marks (factor 0.01) and make subticks so small that they do not appear (factor 1000)
  gr_.SetTickLen(0.01, 1000); 

  // setting font to "none" is necessary to prevent some bugs
  // for more information refer to MathGL's GoogleGroup
  // TODO: find way to successfully load "heros" font. 
  //       (usual 'LoadFont("heros", path)' doens't work properly!)
  gr_.LoadFont("none");
  // set the font size
  gr_.SetFontSizePT(fontSizePT_);

  // Set ranges and call rotate if necessary (to set the correct point of view)
  if (has_3d_){
    // when plotting 3d we do need all the margins and we cannot cut them off 
    // -> cannot call gr_.SubPlot(1,1,0,"<_") or similar here!
    gr_.SetRanges(ranges_[0], ranges_[1], ranges_[2], ranges_[3], zranges_[0], zranges_[1]);
    gr_.Rotate(60, 30);
  }
  else {
    gr_.SubPlot(1, 1, 0, "#"); 
    gr_.SetRanges(ranges_[0], ranges_[1], ranges_[2], ranges_[3]);
  }


  // Add title
  if (title_.size() != 0){
    gr_.Title(title_.c_str());
  }

  // use InPlot to force having a quadratic plot-window
  if (!has_3d_) {
    gr_.InPlot( double(leftMargin_) / figWidth_, // margin from left
                double(leftMargin_ + plotWidth_) / figWidth_, // how far to the right
                double(figHeight_ - plotHeight_ - topMargin_) / figHeight_, // how far to the bottom
                double(figHeight_ - topMargin_) / figHeight_ ); // how far up
    // Set aspects: 1, 1, 1 will give a normal plot. (default)
    //              1,-1, 1 will invert the y axis. (used for spy plots)
    // Note: This *has* to be called after SubPlot and InPlot, otherwise the axis labels will be in 1,1,1 manner
    gr_.Aspect(aspects_[0], aspects_[1], aspects_[2]);
  }

  // Set label - before setting curvilinear because MathGL is vulnerable to errors otherwise
  gr_.Label('x', xMglLabel_.str_.c_str(), xMglLabel_.pos_);
  gr_.Label('y', yMglLabel_.str_.c_str(), yMglLabel_.pos_);

  // Set Curvilinear functions
  gr_.SetFunc(xFunc_.c_str(), yFunc_.c_str(), zFunc_.c_str());

  // Add grid
  if (grid_){
    gr_.Grid(gridType_.c_str() , gridCol_.c_str());
  }

  // Add axis
  if (axis_){
    gr_.Axis();
  }

  // add axis at zero w/o labels if its a barplot
  if (barplot_) {
    // this macro block makes sure that NAN is defined
    # ifndef NAN // check if NAN is already defined
      # if MGL_USE_DOUBLE // if MathGL uses double use the double NAN
        # define NAN (*(double*)mgl_nan)
      # else
        # define NAN (*(float*)(mgl_nan+1))
      # endif
    # endif

    // NAN -> automatically setting axis in x-direction
    gr_.SetOrigin(NAN, 0);
    gr_.Axis("_");
  }

  gr_.Box();
  // Plot
  for (auto &p : plots_) {
    p->plot(&gr_);
  }

  for (auto s : additionalLabels_) {
    gr_.AddLegend(s.first.c_str(), s.second.c_str());
  }

  // Add legend
  if (legend_){
    if (!has_3d_) {
      // scale legend input according to figHeight, figWidth, plotHeight, plotWidth, etc.
      double bx = 1.3*double(leftMargin_)/figWidth_, // helper variables
             by = 1.1*double(topMargin_)/figHeight_;

      if (xMglLabel_.str_.size() != 0) {
        by *= 1.3;
      }

      double newxPos = (1 - 2*bx) * legendPos_.first + bx,
             newyPos = (1 - 2*by) * legendPos_.second + by;
      gr_.Legend(newxPos, newyPos);
    }
    else {
      gr_.Legend(legendPos_.first, legendPos_.second);
    }
  }

#if NDEBUG
  std::cout << "Writing to file ... \n";
#endif

  // Checking if to plot in png or eps and save file
  if (file.find(".png") != std::string::npos){
    gr_.WritePNG(file.c_str());
  }
  else if (file.find(".eps") != std::string::npos){
    gr_.WriteEPS(file.c_str());
  }
  else {
    gr_.WriteEPS((file + ".eps").c_str());
  }
}

} // end namespace mgl
