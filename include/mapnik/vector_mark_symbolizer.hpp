#ifndef VECTOR_MARK_SYMBOLIZER_HPP
#define VECTOR_MARK_SYMBOLIZER_HPP

#include <mapnik/point_symbolizer.hpp>
#include <mapnik/coord.hpp>


namespace mapnik 
{
  struct MAPNIK_DECL vector_mark_symbolizer:
        public symbolizer_with_image, public symbolizer_base
  {
    vector_mark_symbolizer(): 
        symbolizer_with_image(path_expression_ptr(new path_expression)),
        symbolizer_base(),
        base_(0.0) {}
    
    vector_mark_symbolizer(path_expression_ptr filename): 
        symbolizer_with_image(filename),
        symbolizer_base(), 
        base_(0.0) {}
    
    vector_mark_symbolizer(const vector_mark_symbolizer& rhs):
        symbolizer_with_image(rhs),
        symbolizer_base(rhs),
        base_(rhs.base_) {}

    void set_base(double base) { base_ = base; }
    double get_base() const { return base_; }

  private:
      // in pixels
      double base_;
  };
}


#endif 