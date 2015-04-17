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
#include <mapnik/point_symbolizer.hpp>
#include <mapnik/marker.hpp>
#include <mapnik/marker_cache.hpp>
#include <mapnik/label_collision_detector.hpp>
#include <mapnik/parse_path.hpp>
#include <mapnik/pixel_position.hpp>


// agg
#include "agg_trans_affine.h"

// stl
#include <string>

// boost
#include <boost/make_shared.hpp>

namespace mapnik {
    
//TODO Redo!
template <typename T>
inline void render_marker_at_point(agg_renderer<T> &render, double x, double y,
                                   const marker_ptr &marker, const agg::trans_affine &tr,
                                   const point_symbolizer &sym, boost::shared_ptr<label_collision_detector4> detector,
                                   box2d<double> &label)
{
    label.re_center(x, y);
    if(sym.get_allow_overlap() ||
            detector->has_placement(label)) 
    {
        render.render_marker(pixel_position(x, y),
                             *marker,
                             tr,
                             sym.get_opacity(),
                             sym.comp_op());

        if(!sym.get_ignore_placement())
            detector->insert(label);
    }
}

template <typename T>
void agg_renderer<T>::process(point_symbolizer const& sym,
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

        agg::trans_affine tr;
        evaluate_transform(tr, feature, sym.get_image_transform());
        agg::trans_affine_translation recenter(-center.x, -center.y);
        agg::trans_affine recenter_tr = recenter * tr;
        
        box2d<double> label_ext = bbox * recenter_tr * agg::trans_affine_scaling(scale_factor_);

        for (std::size_t i=0; i<feature.num_geometries(); ++i)
        {
            geometry_type const& geom = feature.get_geometry(i);
            geom.rewind(0);
            double x0 = 0.0;
            double y0 = 0.0;
            double x1 = 0.0;
            double y1 = 0.0;
            double z = 0.0;
            if (geom.vertex(&x0, &y0) == SEG_END)
                continue;
            
            if (sym.get_point_placement() == CENTROID_POINT_PLACEMENT)
            {
                if (!label::centroid(geom, x0, y0))
                    return;
                
                prj_trans.backward(x0, y0, z);
                t_.forward(&x0, &y0);
                render_marker_at_point(*this, x0, y0, *marker, tr, sym, detector_, label_ext);
            }
            else if (sym.get_point_placement() == INTERIOR_POINT_PLACEMENT)
            {
                if (!label::interior_position(geom ,x0, y0))
                    return;
                
                prj_trans.backward(x0, y0, z);
                t_.forward(&x0, &y0);
                render_marker_at_point(*this, x0, y0, *marker, tr, sym, detector_, label_ext);
            }
            else if (sym.get_point_placement() == FIRST)
            {
                if (geom.type() == LineString && geom.size() >= 2)
                {
                    prj_trans.backward(x0, y0, z);
                    t_.forward(&x0, &y0);
                    if (sym.get_rotate())
                    {
                        geom.vertex(&x1, &y1);
                        prj_trans.backward(x1, y1, z);
                        t_.forward(&x1, &y1);
                        
                        double length = distance(x0, y0, x1, y1);
                        if (length == 0.0)
                            return;
                        
                        double angle = acos((x1 - x0)/length);
                        if (y1 - y0 < 0.0)
                            angle = -angle;
                        tr *= agg::trans_affine_rotation(angle);
                        label_ext *= agg::trans_affine_rotation(angle);
                    }
                    render_marker_at_point(*this, x0, y0, *marker, tr, sym, detector_, label_ext);
                }
            }
            else if (sym.get_point_placement() == LAST)
            {
                if (geom.type() == LineString && geom.size() >= 2)
                {
                    geom.vertex(geom.size() - 1, &x1, &y1);
                    prj_trans.backward(x1, y1, z);
                    t_.forward(&x1, &y1);
                    if (sym.get_rotate())
                    {
                        prj_trans.backward(x0, y0, z);
                        t_.forward(&x0, &y0);
                        geom.vertex(geom.size() - 2, &x0, &y0);
                        double length = distance(x0, y0, x1, y1);
                        if (length == 0.0)
                            return;
                        
                        double angle = acos((x1 - x0)/length);
                        if (y1 - y0 < 0.0)
                            angle = -angle;
                        tr *= agg::trans_affine_rotation(angle);
                        label_ext *= agg::trans_affine_rotation(angle);
                    }
                    render_marker_at_point(*this, x1, y1, *marker, tr, sym, detector_, label_ext);
                }
            }
            else if (sym.get_point_placement() == ALL)
            {
                double angle = 0.0;
                prj_trans.backward(x0, y0, z);
                t_.forward(&x0, &y0);
                while (SEG_LINETO == geom.vertex(&x1, &y1))
                {
                    prj_trans.backward(x1, y1, z);
                    t_.forward(&x1, &y1);
                    if (sym.get_rotate())
                    {
                        double length = distance(x0, y0, x1, y1);
                        if (length == 0.0)
                            return;
                        angle = acos((x1 - x0)/length);
                        if (y1 - y0 < 0.0)
                            angle = -angle;
                        tr *= agg::trans_affine_rotation(angle);
                        label_ext *= agg::trans_affine_rotation(angle);
                    }
                    render_marker_at_point(*this, x0, y0, *marker, tr, sym, detector_, label_ext);
                    x0 = x1;
                    y0 = y1;
                }
                render_marker_at_point(*this, x0, y0, *marker, tr, sym, detector_, label_ext);
            }
        }
    }

}

template void agg_renderer<image_32>::process(point_symbolizer const&,
                                              mapnik::feature_impl &,
                                              proj_transform const&);

}
