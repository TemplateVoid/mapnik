#ifndef VECTOR_MARK_SYMBOLIZER_HPP
#define VECTOR_MARK_SYMBOLIZER_HPP

#include <mapnik/point_symbolizer.hpp>
#include <mapnik/coord.hpp>


namespace mapnik 
{
  struct MAPNIK_DECL vector_mark_symbolizer:
        public point_symbolizer
  {
    vector_mark_symbolizer(): point_symbolizer(), 
    align_by_geometry_(true),
    stretch_(false),
    base_(0),
    anchor_(0, 0),
    center_(false) {}
    
    vector_mark_symbolizer(path_expression_ptr filename): point_symbolizer(filename), 
    align_by_geometry_(true), 
    stretch_(false), 
    base_(0),
    anchor_(0, 0),
    center_(false) {};
    
    vector_mark_symbolizer(const vector_mark_symbolizer& rhs): point_symbolizer(rhs), 
    align_by_geometry_(rhs.align_by_geometry_), 
    stretch_(rhs.stretch_), 
    base_(rhs.base_),
    anchor_(rhs.anchor_),
    center_(rhs.center_) {}
    
    void set_align_by_geometry(bool is_align) { align_by_geometry_ = is_align; }
    bool get_align_by_geometry() const { return align_by_geometry_; }
    
    void set_stretch( bool stretch) { stretch_ = stretch; }
    bool get_stretch() const { return stretch_; }
    
    void set_base(double base) { base_ = base; }
    double get_base() const { return base_; }
    
    void set_anchor(const coord2d &anchor) { anchor_ = anchor; }
    coord2d get_anchor() const { return anchor_; }
    
    void set_center(bool center) { center_ = center; }
    bool get_center() const { return center_; }
  private:
      bool align_by_geometry_;
      bool stretch_;
      // in pixels
      double base_;
      coord2d anchor_;
      bool center_;
  };
}


#endif 