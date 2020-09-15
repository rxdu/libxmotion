//////////////////////////////////////////////////////////
// (c) Seminar of Applied Mathematics ETH 2016, D-MATH  //
// Author: Julien Gacon <jgacon@ethz.ch>                //
// Co-Author: Baranidharan Mohan                        //
//////////////////////////////////////////////////////////

# ifndef MGL_STYLE_HPP
# define MGL_STYLE_HPP

// system includes
# include <deque>
# include <string>
# include <algorithm>

// MathGL include
# include <mgl2/mgl.h>

namespace mgl {


/* create cross join B x A, such that:
 * A = {0, 1}, B = {a, b, bc} 
 * => C = {0a, 1a, 0b, 1b, 0c, 1c}     */
static void crossjoin (const std::deque<std::string>& A, const std::deque<std::string>& B, std::deque<std::string>& res) {
  for (auto b : B) {
    for (auto a : A) {
      res.push_back(a + b);
    }
  }
}


class MglStyle {
  public: 
    inline void get_new (std::deque<std::string>& new_deque);

    inline MglStyle();

    inline MglStyle(const std::string& already_used);

    template <class Container>
    MglStyle(const Container& already_used);

    inline std::string get_next();

    inline void eliminate (const std::string& already_used);

  private:
    std::deque<std::string> styles_;
};


/* create new style deque                                *
 * PRE : -                                               *
 * POST: new style deque gets *pushed_back* to new_deque */
void MglStyle::get_new (std::deque<std::string>& new_deque) {
  std::deque<std::string> colors = { "b", "r", "g", "c", "m", "y", "G", "p", "q", "k", "n" },
                        linetypes = { "-", ":", ";", "|", "j", "i", "=" };
  crossjoin(colors, linetypes, new_deque);
}


/* constructor: create new styles_ deque with styles from get_new *
 * PRE : -                                                        *
 * POST: new styles get appended to style_ deque                  */
MglStyle::MglStyle() {
  get_new(styles_);
}


/* create new styles_ deque and remove 'already_used'                               *
 * PRE : -                                                                          *
 * POST: new styles get appended to style_ deque and 'already_used' gets eliminated */  
MglStyle::MglStyle (const std::string& already_used) {
  get_new(styles_);
  // if already_used is contained in styles_ remove it
  eliminate(already_used);
}


/* creates new styles_ deque and remove all strings in already_used *
 * PRE : -                                                          *
 * POST: new styles get appended to style_ deque and all styles in  *
 *       already_used_cont get eliminated                           */  
template <class Container>
MglStyle::MglStyle (const Container& already_used_cont) {
 get_new(styles_);

 // iterate over all strings in already_used_cont and remove them 
 for (auto already_used : already_used_cont) {
    eliminate(already_used);
 }
 
 // if all available styles have been used start from the beginning
 if (styles_.size() == 0) {
   get_new(styles_);
 }
}


/* get next style                                           *
 * PRE : -                                                  *
 * POST: returns first element which is in the style_ deque */
std::string MglStyle::get_next() {
  // if all available styles have been used start from the beginning
  if (styles_.size() == 0) {
    get_new(styles_);
  }
  std::string next = styles_[0];
  styles_.pop_front();
  return next;
}


/* sort string to default layout: <solid?><color><styleoption><linewidth>, as in "b:0", "r-1", " r*"                 *
 * if parameters are missing choose default: solid -> "" (true), color -> 'b' , styleoption -> '-', linewidth -> '0' *
 * PRE : style string containing valid characters only (see below or MathGL documentation)                           *
 * POST: string gets re-arranged to a certain order: (solid)(color)(linestyle)(linewidth)                            */
static std::string normalized (const std::string& s) {
    // get parameters
    std::string solid = "",
                color = "b", 
                styleoption = "", 
                linewidth = "0"; 

    auto it = s.begin();

    const std::string valid_colors = "bgrhwBGRHWcmypCMYkPlenuqLENUQ", // colors from MathGL documentation
                      valid_styles = ".+x*sdo^v<>#-|l;=ji: AVKIDSOX_S"; // . to # -- markers, - to : -- linetypes, rest -- arrowstyles

    // checking if solid or not
    if (s[0] == ' ') {
      solid = " ";
      ++it; // first char doesnt need to be checked anymore
    }
    while (it != s.end()) {
# if NDEBUG
      std::cout << "In while loop with it = " << *it << "\n";
# endif
      // checking for numbers (linewidth parameter)
      if (int(*it) >= 48 && int(*it) <= 57) {
        linewidth = *it; 
        ++it; continue;
      }
      // checking for characters (upper and lowercase (color parameter)
      else if (std::find(valid_colors.begin(), valid_colors.end(), *it) != valid_colors.end()) { 
        color = *it;
        ++it; continue;
      }
      // check stylesoptions 
      else if (std::find(valid_styles.begin(), valid_styles.end(), *it) != valid_styles.end()) {
        styleoption += *it; // using += because styles like #+ or #* are valid
        ++it; continue;
      }
      ++it;
    }
    // need to cover the case that no styleoption at all has been found
    if (styleoption.size() == 0) {
      styleoption = "-";
    }

    return (solid + color + styleoption + linewidth);
}


/* eliminates already_used from style_ deque                          *
 * PRE : -                                                            *
 * POST: if style_ contains already_used latter is removed from first */
void MglStyle::eliminate (const std::string& already_used) {

  std::string already_used_normalized = normalized(already_used);

  // we only use linewidth 0 (or 1, they are the same) in the deque, so...
  if (already_used_normalized[2] != '0' && already_used_normalized[2] != '1') {
    return; // if the width is != 0 we dont even have to search for this style in our deque
  } 
  else {
    already_used_normalized.erase(2); // if it is 0 we have to look for it
  }

  // we only must check for solid styles
  if (already_used_normalized[0] == ' ') {
    return;
  }

  auto it = std::find(styles_.begin(), styles_.end(), already_used_normalized);
  if (it != styles_.end()) {
    styles_.erase(it);
  }
}


} // end namespace mgl


# endif // MGL_STYLE_HPP
