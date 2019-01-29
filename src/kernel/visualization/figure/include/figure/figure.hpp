//////////////////////////////////////////////////////////
// (c) Seminar of Applied Mathematics ETH 2016, D-MATH  //
// Author: Julien Gacon <jgacon@ethz.ch>                //
// Co-Author: Baranidharan Mohan                        //
//////////////////////////////////////////////////////////

# ifndef FIGURE_HPP
# define FIGURE_HPP

// system includes
# include <iostream>
# include <memory>
# include <array>
# include <utility>
# include <stdexcept>
# include <cassert>
# include <numeric>
# include <cctype> // isalpha

// Eigen includes
# include "FigureConfig.hpp"
# if FIG_HAS_EIGEN
  # include <Eigen/Dense>
  # include <Eigen/Sparse>
# endif

// own includes 
# include "MglPlot.hpp"
# include "MglLabel.hpp"
# include "MglStyle.hpp"
// MathGL include
# include <mgl2/mgl.h>

namespace mgl {

/*******************************************************************
 *                    Universal functions                          *
 *******************************************************************/


/* make mglData from std::vector                                    *
 * PRE: -                                                           *
 * POST: returning mglData containing the data of given std::vector */
template <typename Scalar>
typename std::enable_if<std::is_arithmetic<Scalar>::value, mglData>::type
make_mgldata(const std::vector<Scalar>& v) {
  std::vector<double> vd(v.begin(), v.end());
  return mglData(vd.data(), vd.size());
}

/* make mglData from Eigen::Vector or Eigen::RowVector                     *
 * PRE : -                                                                 *
 * POST: returning mglData containing the data of given Eigen::(Row)Vector */
# if FIG_HAS_EIGEN
template <typename Derived>
mglData make_mgldata(const Eigen::MatrixBase<Derived>& vec) {
  if (! (vec.rows() == 1 || vec.cols() == 1) ) {
    std::cerr << "In function Figure::make_mgldata(): vector.cols() == 1 || vector.rows() == 1 failed!";
    assert(vec.rows() == 1 || vec.cols() == 1);
  }
  std::vector<typename Derived::Scalar> v;
  if(vec.rows() == 1) {
    v.resize(vec.cols());
    for(int i = 0; i < vec.cols(); i++)
      v[i] = vec[i];
  } else {
    v.resize(vec.rows());
    for(int i = 0; i < vec.rows(); i++)
      v[i] = vec[i];
  }
  return mglData(v.data(), v.size());
}

/* make mglData from Eigen::Array                                    *
 * PRE : -                                                           *
 * POST: returning mglData containing the data of given Eigen::Array */
template<typename Derived>
mglData make_mgldata(const Eigen::ArrayBase<Derived>& a) {
  return make_mgldata(a.matrix());
}
# endif


/*******************************************************************
 *                 Declaration of Figure class                     *
 *******************************************************************/


class Figure {
public:
  Figure();

  void setRanges(const mglData& xd, const mglData& yd, double vertMargin = 0.1);

  void setRanges(const mglData& xd, const mglData& yd, const mglData& zd);

  void grid(bool on = true, const std::string& gridType = "-", const std::string& gridCol = "h");

  void xlabel(const std::string& label, double pos = 0);

  void ylabel(const std::string& label, double pos = 0);

  void legend(const double& xPos = 1, const double& yPos = 1);

  void addlabel(const std::string& label, const std::string& style);

  template <typename Matrix>
  MglPlot& bar(const Matrix& y, std::string style = "");

  // Note: this tedious template expression ensures y is a Matrix and not e.g. a string, which could happen
  //       as the argument is templated
  template <typename xVector, typename Matrix>
  typename std::enable_if<!std::is_same<typename std::remove_pointer<typename std::decay<Matrix>::type>::type, char >::value, MglPlot&>::type
  bar(const xVector& x, const Matrix& y, std::string style = "");

  template <typename yVector>
  MglPlot& plot(const yVector& y, std::string style = "");

  template <typename xVector, typename yVector>
  typename std::enable_if<!std::is_same<typename std::remove_pointer<typename std::decay<yVector>::type>::type, char >::value, MglPlot&>::type
  plot(const xVector& x, const yVector& y, std::string style = "");

  template <typename xVector, typename yVector, typename zVector>
  MglPlot& plot3(const xVector& x, const yVector& y, const zVector& z, std::string style = "");

  MglPlot& fplot(const std::string& function, std::string style = "");

# if FIG_HAS_EIGEN
  template <typename Scalar, typename xVector, typename yVector>
  MglPlot& triplot(const Eigen::Matrix<Scalar, -1, -1, Eigen::ColMajor>& T, const xVector& x, const yVector& y, std::string style = "");

  template <typename Scalar, typename xVector, typename yVector>
  MglPlot& triplot(const Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor>& T, const xVector& x, const yVector& y, std::string style = "");
# endif

  void ranges(const double& xMin, const double& xMax, const double& yMin, const double& yMax);

  void save(const std::string& file);

  void setlog(bool logx = false, bool logy = false, bool logz = false);

  void setPlotHeight(const int height);

  void setPlotWidth(const int width);

  void setTopMargin(const int top);

  void setLeftMargin(const int left);

  void setHeight(const int height);

  void setWidth(const int width);

  void setFontSize(const int size);

  template <typename Matrix> // dense version
  MglPlot& spy(const Matrix& A, const std::string& style = "b");

# if FIG_HAS_EIGEN // only enable if Eigen is available, otherwise Eigen::SparseMatrix will not be defined
  template <typename Scalar> // sparse version
  MglPlot& spy(const Eigen::SparseMatrix<Scalar>& A, const std::string& style = "b");
# endif

  void title(const std::string& text);

private:
  bool autoRanges_; // auto ranges or ranges as the user set them?
  bool axis_; // plot axis?
  bool barplot_; // containing barplot? (needed to set axis properly)
  bool grid_; // plot grid?
  bool has_3d_; // are there 3d plots?
  bool legend_; // plot legend

  int figHeight_, figWidth_; // height and width of the whole image
  int plotHeight_, plotWidth_; // height and width of the plot
  int leftMargin_, topMargin_; // left and top margin of plot inside the image

  double fontSizePT_; // font size in PT

  std::string gridCol_; // grid color
  std::string gridType_; // grid type
  std::string title_; // title of the plot
  std::string xFunc_, yFunc_, zFunc_; // curvature of coordinate axis

  std::pair<double, double> legendPos_; // legend position

  std::array<double, 3> aspects_; // axis aspects, e.g. -1 used to invert axis. see MathGL documentation
  std::array<double, 4> ranges_; // axis ranges
  std::array<double, 2> zranges_; // z axis ranges

  MglLabel xMglLabel_, yMglLabel_; // x and y labels of the plot
  MglStyle styles_; // styles of the plots

  std::vector<std::unique_ptr<MglPlot> > plots_; // x, y (and z) data for the plots
  std::vector<std::pair<std::string, std::string>> additionalLabels_; // manually added labels 
};


/*******************************************************************
 *                  Templated member functions                     *
 *******************************************************************/


/* bar plot for given y data                                                       *
 * PRE : -                                                                         *
 * POST: add bar plot of [1:length(y)]-y to plot queue with given style (optional) */
template <typename Matrix>
MglPlot& Figure::bar(const Matrix& y, std::string style) {
  // build a fitting x vector for the y vector
  std::vector<double> x(y.rows());
  std::iota(x.begin(), x.end(), 1);
  return bar(x, y, style);
}

/* bar plot of x,y data                                                *
 * PRE : -                                                             *
 * POST: add bar plot of x-y to plot queue with given style (optional) */
template <typename xVector, typename Matrix>
// the long template magic expression ensures this function is not called if yVector is a string (which would be allowed as it is a templated argument)
typename std::enable_if<!std::is_same<typename std::remove_pointer<typename std::decay<Matrix>::type>::type, char >::value, MglPlot&>::type
Figure::bar(const xVector& x, const Matrix& y, std::string style) {

  // TODO check if x is really a vector. Problem: cant use .rows or .cols as it can be a std::vector
  
  // set flag for barplots. this will set the y-axis origin to 0 later
  barplot_ = true;

  const long m = y.rows(),
             n = y.cols();

  // check that input lengths fit
  if (x.size() != m) {
    std::cerr << "In function Figure::bar(): Data sets must have same lengths! Remember to save the bar-data in columns!";
  }

  // initialize data
  mglData xd = make_mgldata(x); // straightforward for x vector
  mglData yd = mglData(y.cols(), y.rows(), y.data()); // y might be a matrix

  // if the ranges are set to auto set the new ranges 
  if(autoRanges_){
    setRanges(xd, yd, 0.); // the 0 stands no top+bottom margin
  }
  
  // check if a style is given,
  // if yes: add that style to the style queue and remove it from the style-deque,
  // if no : get a new style from the style-deque
  if (style.size() == 0) {
    for (long i = 0; i < n; ++i) {
      style += styles_.get_next();
    }
  }
  else {
    // special characters dont matter in bar plot, remove all non-alphabetic characters
    std::remove_if(style.begin(), style.end(), [](int ch){ return !std::isalpha(ch); });
    // eliminate all used colors from styles
    for (std::size_t j = 0; j < style.size(); ++j) {
      std::stringstream ss;
      ss << style[j];
      styles_.eliminate(ss.str());
    }
  }

  // put the x-y data in the plot queue 
  plots_.emplace_back(std::unique_ptr<MglBarPlot>(new MglBarPlot(xd, yd, style)));
  return *plots_.back().get();
}

/* plot y data                                                         *
 * PRE : -                                                             *
 * POST: add [1:length(y)]-y to plot queue with given style (optional) */
template <typename yVector>
MglPlot& Figure::plot(const yVector& y, std::string style) {
  // build a fitting x vector for the y vector
  std::vector<double> x(y.size());
  std::iota(x.begin(), x.end(), 1);
  return plot(x, y, style);
}

/* plot x,y data                                           *
 * PRE : -                                                 *
 * POST: add x-y to plot queue with given style (optional) */
template <typename xVector, typename yVector>
// the long template magic expression ensures this function is not called if yVector is a string (which would be allowed as it is a templated argument)
typename std::enable_if<!std::is_same<typename std::remove_pointer<typename std::decay<yVector>::type>::type, char >::value, MglPlot&>::type
Figure::plot(const xVector& x, const yVector& y, std::string style) {
  // make sure the sizes of the vectors are the same
  if (x.size() != y.size()){
    std::cerr << "In function Figure::plot(): Vectors must have same sizes!";
  }

  // build mglData from the x and y vectors
  mglData xd = make_mgldata(x);
  mglData yd = make_mgldata(y);

  // if the ranges are set to auto set the new ranges 
  if(autoRanges_){
    setRanges(xd, yd, 0.); // the 0 stands no top+bottom margin
  }
  
  // check if a style is given,
  // if yes: add that style to the style queue and remove it from the style-deque,
  // if no : get a new style from the style-deque
  if (style.size() == 0) {
    style = styles_.get_next();
  }
  else {
    styles_.eliminate(style);
  }

  // put the x-y data in the plot queue
  plots_.emplace_back(std::unique_ptr<MglPlot2d>(new MglPlot2d(xd, yd, style)));
  return *plots_.back().get();
}

/* plot x,y,z data                                           *
 * PRE : -                                                   *
 * POST: add x-y-z tp plot queue with given style (optional) */
template <typename xVector, typename yVector, typename zVector>
MglPlot& Figure::plot3(const xVector& x, const yVector& y, const zVector& z, std::string style) {

  // needed to set zranges in save-function and call mgl::Rotate
  has_3d_ = true; 

  // make sure the sizes of the vectors are the same
  if (!(x.size() == y.size() && y.size() == z.size())){
    std::cerr << "In function Figure::plot(): Vectors must have same sizes!";
  }

  // build mglData from x, y and z vectors
  mglData xd = make_mgldata(x);
  mglData yd = make_mgldata(y);
  mglData zd = make_mgldata(z);

  // if the ranges are set to auto set the new ranges
  if(autoRanges_){
    setRanges(xd, yd, zd);
  }

  // check if a style is given,
  // if yes: add that style to the style queue and remove it from the style-deque,
  // if no : get a new style from the style-deque
  if (style.size() == 0) {
    style = styles_.get_next();
  }
  else {
    styles_.eliminate(style);
  }

  // put the x-y-z data in the plot queue
  plots_.emplace_back(std::unique_ptr<MglPlot3d>(new MglPlot3d(xd, yd, zd, style)));
  return *plots_.back().get();
}

template <typename Matrix>
MglPlot& Figure::spy(const Matrix& A, const std::string& style) {

  has_3d_ = false;
  aspects_[1] = -1; // invert y-axis

  // determine radius of dots
  std::string radius = "8";
  if (std::max(A.cols(), A.rows()) > 99) {
    radius = "5";
  }
  if (std::max(A.cols(), A.rows()) > 999) {
    radius = "3";
  }
  if (std::max(A.cols(), A.rows()) > 9999) {
    radius = "1";
  }
  
  // counting nonzero entries
  unsigned long counter = 0;
  // save positions of entries in these vectors
  // x for the col-index and y for the row-index
  std::vector<double> x, y;

  ranges_ = std::array<double, 4>{0, (static_cast<double>(A.cols() + 1)), 0,
				  (static_cast<double>(A.rows() + 1)) };
  for (unsigned i = 0; i < A.rows(); ++i) {
    for (unsigned j = 0; j < A.cols(); ++j) {
      if (A(i,j) != 0) {
        ++counter;
        x.push_back(j + 1);
        // if the row is zero plot at the top, not bottom
        y.push_back(i + 1);
      }
    }
  }
  mglData xd(x.data(), x.size()),
          yd(y.data(), y.size());

  std::stringstream label;
  label << "nnz = ";
  label << counter;
  xMglLabel_ = MglLabel(label.str());

  plots_.emplace_back(std::unique_ptr<MglSpy>(new MglSpy(xd, yd, style + radius)));
  return *plots_.back().get();
}

# if FIG_HAS_EIGEN
template <typename Scalar> 
MglPlot& Figure::spy(const Eigen::SparseMatrix<Scalar>& A, const std::string& style) {

   has_3d_ = false;
   aspects_[1] = -1; // invert y-axis

  // determine radius of dots
  std::string radius = "8";
  if (std::max(A.cols(), A.rows()) > 99) {
    radius = "5";
  }
  if (std::max(A.cols(), A.rows()) > 999) {
    radius = "3";
  }
  if (std::max(A.cols(), A.rows()) > 9999) {
    radius = "1";
  }
  
  // counting nonzero entries
  unsigned long counter = 0;
  // save positions of entries in these vectors
  std::vector<double> x, y;

  ranges_ = std::array<double, 4>{0, (static_cast<double>(A.cols() + 1)), 0,
				  (static_cast<double>(A.rows() + 1)) };
  // iterate over nonzero entries, using method suggested in Eigen::Sparse documentation
  for (unsigned k = 0; k < A.outerSize(); ++k) {
    for (typename Eigen::SparseMatrix<Scalar>::InnerIterator it(A, k); it; ++it) {
      ++counter;
      x.push_back( it.col() + 1 );
      y.push_back( it.row() + 1 );
    }
  }
  mglData xd(x.data(), x.size()),
          yd(y.data(), y.size());

  std::stringstream label;
  label << "nnz = ";
  label << counter;
  xMglLabel_ = MglLabel(label.str());

  plots_.emplace_back(std::unique_ptr<MglSpy>(new MglSpy(xd, yd, style + radius)));
  return *plots_.back().get();
}

template <typename Scalar, typename xVector, typename yVector>
MglPlot& Figure::triplot(const Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor>& T, const xVector& x, const yVector& y, std::string style) {
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> T_double = T.template cast<double>();

  mglData Td(T_double.rows(), T_double.cols(), T_double.data()),
          xd(x.data(), x.size()),
          yd(y.data(), y.size());

  bool draw_numbers = false; // draw numbers on the vertices

  // if the ranges are set to auto set the new ranges 
  if(autoRanges_){
    setRanges(xd, yd, 0.); // the 0 stands no top+bottom margin
  }
  
  // check if a style is given,
  // if yes: add that style to the style queue and remove it from the style-deque,
  // if no : get a new style from the style-deque
  // Note: '#' is to draw lines and not a fully colored plot,
  //       if the style contains 'F' then the triplot will be fully colored
  if (style.size() == 0) {
    style = styles_.get_next() + "#";
  }
  else {
    styles_.eliminate(style);
    // if no 'F' is found add a '#'
    if ( std::find(style.begin(), style.end(), 'F') == style.end() ) {
      style += "#";
    }

    std::string::size_type enumerate = style.find('?');
    if (enumerate != std::string::npos) {
      draw_numbers = true;
      //style.erase(enumerate);
    }
  }

  // put the x-y data in the plot queue
  plots_.emplace_back(std::unique_ptr<MglTriPlot>(new MglTriPlot(Td, xd, yd, style, draw_numbers)));
  return *plots_.back().get();
}

template <typename Scalar, typename xVector, typename yVector>
MglPlot& Figure::triplot(const Eigen::Matrix<Scalar, -1, -1, Eigen::ColMajor>& T, const xVector& x, const yVector& y, std::string style) {
  Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> TRow(T);
  return triplot(TRow, x, y, style);
}

# endif

} // end namespace

# endif // FIGURE_HPP
