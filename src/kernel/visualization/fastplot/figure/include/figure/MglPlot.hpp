//////////////////////////////////////////////////////////
// (c) Seminar of Applied Mathematics ETH 2016, D-MATH  //
// Author: Julien Gacon <jgacon@ethz.ch>                //
// Co-Author: Baranidharan Mohan                        //
//////////////////////////////////////////////////////////

# ifndef MGL_PLOT_HPP
# define MGL_PLOT_HPP

// system includes
# include <iostream>

// MathGL include
# include <mgl2/mgl.h>

namespace mgl {

class MglPlot {
public:

  MglPlot(const std::string& style)
    : style_(style),
      legend_("")
  {}

  virtual void plot(mglGraph* gr) = 0;
  virtual bool is_3d() = 0;

  MglPlot& label(const std::string& l) {
    legend_ = l;
    return *this;
  }

  MglPlot& style(const std::string& s) {
    style_ = s;
    return *this;
  }

  MglPlot& width(int w) {
    w = w < 0 ? 0 : (w > 9 ? 9 : w);
    if(style_.size() == 3)
      style_[3] = char( '0' + w );
    else
      style_ += {char( '0' + w )};
    return *this;
  }

protected:
  std::string style_;
  std::string legend_;
};


class MglPlot2d : public MglPlot {
public:

  MglPlot2d(const mglData& xd, const mglData& yd, const std::string& style)
    : MglPlot(style),
      xd_(xd),
      yd_(yd)
  {}

  void plot(mglGraph* gr) {
    gr->Plot(xd_, yd_, style_.c_str());
    // only add the legend-entry if there is one, otherwise we might end up
    // with a legend-entry containing the line style but no description
    if (legend_.size() > 0) { 
      gr->AddLegend(legend_.c_str(), style_.c_str());
    }
  }

  bool is_3d() {
    return false;
  }

private:
  mglData xd_;
  mglData yd_;
};


class MglPlot3d : public MglPlot {
public:

  MglPlot3d(const mglData& xd, const mglData& yd, const mglData& zd, const std::string& style)
    : MglPlot(style),
      xd_(xd),
      yd_(yd),
      zd_(zd)
  {}

  void plot(mglGraph* gr) {
    gr->Plot(xd_, yd_, zd_, style_.c_str());
    // only add the legend-entry if there is one, otherwise we might end up
    // with a legend-entry containing the line style but no description
    if (legend_.size() > 0) { 
      gr->AddLegend(legend_.c_str(), style_.c_str());
    }
  }

  bool is_3d() {
    return true;
  }

private:
  mglData xd_;
  mglData yd_;
  mglData zd_;
};


class MglFPlot : public MglPlot {
public:

  MglFPlot(const std::string& fplot_str, const std::string& style)
    : MglPlot(style),
      fplot_str_(fplot_str)
  {}

  void plot(mglGraph* gr) {
    gr->FPlot(fplot_str_.c_str(), style_.c_str());
    // only add the legend-entry if there is one, otherwise we might end up
    // with a legend-entry containing the line style but no description
    if (legend_.size() > 0) { 
      gr->AddLegend(legend_.c_str(), style_.c_str());
    }
  }

  bool is_3d() {
    return false;
  }

private:
  std::string fplot_str_;
};


class MglSpy : public MglPlot {
public:

  MglSpy(const mglData& xd, const mglData& yd, const std::string& style) 
    : MglPlot(style),
      xd_(xd),
      yd_(yd)
  {}

  bool is_3d() {
    return false;
  }

  void plot(mglGraph* gr) {
    mglData zd(xd_);
    zd.Modify("0");
    gr->Dots(xd_, yd_, zd, style_.c_str());
  }

private:
  mglData xd_;
  mglData yd_;
};


class MglBarPlot : public MglPlot {
public:
  MglBarPlot(const mglData& xd, const mglData& yd, const std::string& style) 
    : MglPlot(style),
      xd_(xd),
      yd_(yd)
  {}

  bool is_3d() {
    return false;
  }

  void plot(mglGraph* gr) {
    gr->Bars(xd_, yd_, style_.c_str());
    // only add the legend-entry if there is one, otherwise we might end up
    // with a legend-entry containing the line style but no description
    if (legend_.size() > 0) { 
      gr->AddLegend(legend_.c_str(), style_.c_str());
    }
  }

private:
  mglData xd_;
  mglData yd_;
};


class MglTriPlot : public MglPlot {
public:
  MglTriPlot(const mglData& Td, const mglData& xd, const mglData& yd, const std::string& style, const bool draw_numbers) 
    : MglPlot(style),
      Td_(Td),
      xd_(xd),
      yd_(yd),
      draw_numbers_(draw_numbers)
  {}

  bool is_3d() {
    return false;
  }

  void plot(mglGraph* gr) {
    gr->TriPlot(Td_, xd_, yd_, style_.c_str());
    if (legend_.size() > 0) {
      gr->AddLegend(legend_.c_str(), style_.c_str());
    }
    if (draw_numbers_) {
      gr->Label(xd_, yd_, "%n");
    }
  }

private:
  mglData Td_;
  mglData xd_;
  mglData yd_;
  bool draw_numbers_;
};


} // end namespace

# endif // MGL_PLOT_HPP
