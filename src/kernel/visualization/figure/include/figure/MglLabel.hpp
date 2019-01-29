//////////////////////////////////////////////////////////
// (c) Seminar of Applied Mathematics ETH 2016, D-MATH  //
// Author: Julien Gacon <jgacon@ethz.ch>                //
// Co-Author: Baranidharan Mohan                        //
//////////////////////////////////////////////////////////

# ifndef MGL_LABEL_HPP
# define MGL_LABEL_HPP

// MathGL include
# include <mgl2/mgl.h>

namespace mgl {


struct MglLabel {
  MglLabel(const std::string& str = "", const double pos = 0)
    : str_(str),
      pos_(pos)
  {}

  std::string str_;
  double pos_;
};


}

# endif // MGL_LABEL_HPP
