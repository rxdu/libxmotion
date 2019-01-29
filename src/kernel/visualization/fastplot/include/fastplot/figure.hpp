/* 
 * figure.hpp
 * 
 * Created on: Jan 29, 2019 00:55
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef FIGURE_HPP
#define FIGURE_HPP

#include <string>
#include <array>
#include <memory>

#include <mgl2/mgl.h>

namespace librav
{
class Figure
{
  public:
    Figure();

    void setRanges(const mglData &xd, const mglData &yd, double vertMargin = 0.1);

    void setRanges(const mglData &xd, const mglData &yd, const mglData &zd);

    void grid(bool on = true, const std::string &gridType = "-", const std::string &gridCol = "h");

    void xlabel(const std::string &label, double pos = 0);

    void ylabel(const std::string &label, double pos = 0);

    void legend(const double &xPos = 1, const double &yPos = 1);

    void addlabel(const std::string &label, const std::string &style);

    template <typename Matrix>
    MglPlot &bar(const Matrix &y, std::string style = "");

    // Note: this tedious template expression ensures y is a Matrix and not e.g. a string, which could happen
    //       as the argument is templated
    template <typename xVector, typename Matrix>
    typename std::enable_if<!std::is_same<typename std::remove_pointer<typename std::decay<Matrix>::type>::type, char>::value, MglPlot &>::type
    bar(const xVector &x, const Matrix &y, std::string style = "");

    template <typename yVector>
    MglPlot &plot(const yVector &y, std::string style = "");

    template <typename xVector, typename yVector>
    typename std::enable_if<!std::is_same<typename std::remove_pointer<typename std::decay<yVector>::type>::type, char>::value, MglPlot &>::type
    plot(const xVector &x, const yVector &y, std::string style = "");

    template <typename xVector, typename yVector, typename zVector>
    MglPlot &plot3(const xVector &x, const yVector &y, const zVector &z, std::string style = "");

    MglPlot &fplot(const std::string &function, std::string style = "");

#if FIG_HAS_EIGEN
    template <typename Scalar, typename xVector, typename yVector>
    MglPlot &triplot(const Eigen::Matrix<Scalar, -1, -1, Eigen::ColMajor> &T, const xVector &x, const yVector &y, std::string style = "");

    template <typename Scalar, typename xVector, typename yVector>
    MglPlot &triplot(const Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> &T, const xVector &x, const yVector &y, std::string style = "");
#endif

    void ranges(const double &xMin, const double &xMax, const double &yMin, const double &yMax);

    void save(const std::string &file);

    void setlog(bool logx = false, bool logy = false, bool logz = false);

    void setPlotHeight(const int height);

    void setPlotWidth(const int width);

    void setTopMargin(const int top);

    void setLeftMargin(const int left);

    void setHeight(const int height);

    void setWidth(const int width);

    void setFontSize(const int size);

    template <typename Matrix> // dense version
    MglPlot &spy(const Matrix &A, const std::string &style = "b");

#if FIG_HAS_EIGEN              // only enable if Eigen is available, otherwise Eigen::SparseMatrix will not be defined
    template <typename Scalar> // sparse version
    MglPlot &spy(const Eigen::SparseMatrix<Scalar> &A, const std::string &style = "b");
#endif

    void title(const std::string &text);

  private:
    bool autoRanges_; // auto ranges or ranges as the user set them?
    bool axis_;       // plot axis?
    bool barplot_;    // containing barplot? (needed to set axis properly)
    bool grid_;       // plot grid?
    bool has_3d_;     // are there 3d plots?
    bool legend_;     // plot legend

    int figHeight_, figWidth_;   // height and width of the whole image
    int plotHeight_, plotWidth_; // height and width of the plot
    int leftMargin_, topMargin_; // left and top margin of plot inside the image

    double fontSizePT_; // font size in PT

    std::string gridCol_;               // grid color
    std::string gridType_;              // grid type
    std::string title_;                 // title of the plot
    std::string xFunc_, yFunc_, zFunc_; // curvature of coordinate axis

    std::pair<double, double> legendPos_; // legend position

    std::array<double, 3> aspects_; // axis aspects, e.g. -1 used to invert axis. see MathGL documentation
    std::array<double, 4> ranges_;  // axis ranges
    std::array<double, 2> zranges_; // z axis ranges

    MglLabel xMglLabel_, yMglLabel_; // x and y labels of the plot
    MglStyle styles_;                // styles of the plots

    std::vector<std::unique_ptr<MglPlot>> plots_;                       // x, y (and z) data for the plots
    std::vector<std::pair<std::string, std::string>> additionalLabels_; // manually added labels
};
} // namespace librav

#endif /* FIGURE_HPP */
