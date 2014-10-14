/*****************************************************************************
 *
 * This file is part of Mapnik (c++ mapping toolkit)
 *
 * Copyright (C) 2011 Artem Pavlenko
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *****************************************************************************/

// mapnik
#include <mapnik/feature.hpp>
#include <mapnik/agg_renderer.hpp>
#include <mapnik/agg_rasterizer.hpp>
#include <mapnik/image_util.hpp>

#include <mapnik/geom_util.hpp>
#include <mapnik/vector_mark_symbolizer.hpp>
#include <mapnik/marker.hpp>
#include <mapnik/marker_cache.hpp>
#include <mapnik/label_collision_detector.hpp>
#include <mapnik/parse_path.hpp>
#include <mapnik/pixel_position.hpp>
#include <mapnik/coord.hpp>


// agg
#include "agg_trans_affine.h"

// stl
#include <string>
#include <cmath>

// boost
#include <boost/make_shared.hpp>

namespace mapnik {
  

template <typename T>
void agg_renderer<T>::process(vector_mark_symbolizer const& sym,
                              mapnik::feature_impl & feature,
                              proj_transform const& prj_trans)
{
    std::string filename = path_processor_type::evaluate(*sym.get_filename(), feature);

    boost::optional<mapnik::marker_ptr> marker;
    if ( !filename.empty() )
    {
        marker = marker_cache::instance().find(filename, true);
    }
    else
    {
        marker.reset(boost::make_shared<mapnik::marker>());
    }

    if (marker)
    {
        box2d<double> const& bbox = (*marker)->bounding_box();
        coord2d center = bbox.center();
        coord2d offset = sym.get_anchor();

        agg::trans_affine tr;
        evaluate_transform(tr, feature, sym.get_image_transform());
        agg::trans_affine_translation recenter(-offset.x, -offset.y);
	tr *= agg::trans_affine_translation(center.x, center.y);
	tr *= recenter;
        
        for (std::size_t i=0; i<feature.num_geometries(); ++i)
        {
            geometry_type const& geom = feature.get_geometry(i);
            if (!geom.size())
                continue;
            
            double x0,y0;            
            geom.vertex(0, &x0, &y0);
            t_.forward(&x0, &y0);
            
            if (sym.get_align_by_geometry() && geom.size() >= 2) 
            {
                double x1, y1;
                geom.vertex(1, &x1, &y1);
                t_.forward(&x1, &y1);
                if (x1 - x0 == 0 && y1 - y0 == 0)
                    continue;
                
                double length = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
                double angle = acos((x1-x0)/length);
                if (y1 -y0 < 0)
                    angle = -angle;
        
                agg::trans_affine_rotation rotation(angle);
                
                if (sym.get_stretch()) 
                {
                    double scale = length/sym.get_base();
                    tr *= agg::trans_affine_scaling(scale, 1.0);
                }
                tr *= rotation;
            }
            
            
            box2d<double> label_ext = bbox * recenter * tr * agg::trans_affine_scaling(scale_factor_);
            
            if (sym.get_center())
            {
                if (!label::centroid(geom, x0, y0))
                    return;
                t_.forward(&x0, &y0);
            }
                       
            
            label_ext *= agg::trans_affine_translation(x0, y0);
            
            if (sym.get_allow_overlap() ||
                detector_->has_placement(label_ext))
            {

                render_marker(pixel_position(x0, y0),
                              **marker,
                              tr,
                              sym.get_opacity(),
                              sym.comp_op());

                if (!sym.get_ignore_placement())
                    detector_->insert(label_ext);
            }
        }
    }
}

template void agg_renderer<image_32>::process(vector_mark_symbolizer const&,
                                              mapnik::feature_impl &,
                                              proj_transform const&);

}
