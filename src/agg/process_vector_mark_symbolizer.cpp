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

    if (marker && marker.get()->is_vector())
    {
        box2d<double> const& bbox = (*marker)->bounding_box();
        coord2d center = bbox.center();

        agg::trans_affine tr;
        evaluate_transform(tr, feature, sym.get_image_transform());
        tr *= agg::trans_affine_translation(center.x, center.y);
        
        for (std::size_t i=0; i<feature.num_geometries(); ++i)
        {
            geometry_type const& geom = feature.get_geometry(i);
            geom.rewind(0);
            double x0 = 0.0;
            double y0 = 0.0;
            double x1 = 0.0;
            double y1 = 0.0;
            double z = 0.0;
            unsigned command = geom.vertex(&x0, &y0);
            if (command == SEG_END)
                continue;
            while (SEG_LINETO == geom.vertex(&x1, &y1))
            {
                prj_trans.backward(x0, y0, z);
                t_.forward(&x0, &y0);
                prj_trans.backward(x1, y1, z);
                t_.forward(&x1, &y1);
                
                agg::trans_affine trans = tr;
                
                double length = distance(x0, y0, x1, y1);
                if (length == 0.0)
                    continue;
                
                double scale = length/(sym.get_base()*scale_factor_);
                trans *= agg::trans_affine_scaling(scale);
                
                double angle = acos((x1 - x0)/length);
                if (y1 - y0 < 0.0)
                    angle = -angle;
                trans *= agg::trans_affine_rotation(angle);
                render_marker(pixel_position(x0, y0), **marker,
                              trans, sym.get_opacity(), sym.comp_op());
                x0 = x1;
                y0 = y1;
            }
        }
    }
}

template void agg_renderer<image_32>::process(vector_mark_symbolizer const&,
                                              mapnik::feature_impl &,
                                              proj_transform const&);

}
