/*****************************************************************************
 *
 * This file is part of Mapnik (c++ mapping toolkit)
 *
 * Copyright (C) 2014 Artem Pavlenko
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
#include <mapnik/group/group_symbolizer_helper.hpp>
#include <mapnik/label_collision_detector.hpp>
#include <mapnik/geom_util.hpp>
#include <mapnik/debug.hpp>
#include <mapnik/symbolizer.hpp>
#include <mapnik/value_types.hpp>
#include <mapnik/text/placements/base.hpp>
#include <mapnik/text/placements/dummy.hpp>
#include <mapnik/vertex_cache.hpp>
#include <mapnik/tolerance_iterator.hpp>
#include <mapnik/edge_placement_adjuster.h>

//agg
#include "agg_conv_clip_polyline.h"

namespace mapnik {

group_symbolizer_helper::group_symbolizer_helper(
        group_symbolizer const& sym, feature_impl const& feature,
        attributes const& vars,
        proj_transform const& prj_trans,
        unsigned width, unsigned height, double scale_factor,
        view_transform const& t, DetectorType & detector,
        box2d<double> const& query_extent)
    : base_symbolizer_helper(sym, feature, vars, prj_trans, width, height, scale_factor, t, query_extent),
      detector_(detector)
{}

pixel_position_list const& group_symbolizer_helper::get()
{
    results_.clear();

    if (point_placement_)
    {
        for (pixel_position const& point : points_)
        {
            check_point_placement(point);
        }
    }
    else
    {
        for (auto const& geom : geometries_to_process_)
        {
            // TODO to support clipped geometries this needs to use
            // vertex_converters
            using path_type = transform_path_adapter<view_transform,geometry_type>;
            path_type path(t_, *geom, prj_trans_);
            find_line_placements(path);
        }
    }

    return results_;
}

template <typename T>
bool group_symbolizer_helper::find_line_placements(T & path)
{
    if (box_elements_.empty()) return true;

    vertex_cache pp(path);

    bool success = false;
    while (pp.next_subpath())
    {
        if (pp.length() <= 0.001)
        {
            success = check_point_placement(pp.current_position()) || success;
            continue;
        }

        double spacing = get_spacing(pp.length());

        pp.forward(spacing/2.0);
        do
        {
            tolerance_iterator tolerance_offset(text_props_->label_position_tolerance * scale_factor_, spacing); //TODO: Handle halign
            while (tolerance_offset.next())
            {
                vertex_cache::scoped_state state(pp);
                if (pp.move(tolerance_offset.get()) && check_point_placement(pp.current_position()))
                {
                    success = true;
                    break;
                }
            }
        } while (pp.forward(spacing));
    }
    return success;
}

bool group_symbolizer_helper::check_point_placement(pixel_position const& pos)
{
    if (box_elements_.empty()) return false;
    double x = pos.x;
    double y = pos.y;
    // offset boxes and check collision
    std::list< box2d<double> > real_boxes;
    for (auto const& box_elem : box_elements_)
    {
        box2d<double> real_box = box2d<double>(box_elem.box_);
        real_box.move(pos.x, pos.y);
        edge_placement_adjuster adjuster;
        if(text_props_->adjust_edges &&
            adjuster.adjust_edge_placement(agg::trans_affine(), box_elem.box_, detector_.extent(), real_box, x, y) )
        {
            real_box = box2d<double>(box_elem.box_);
            real_box.move(x, y);
            if(collision(real_box, box_elem.repeat_key_))
            {
                //the box was moved but still not placed within bounds
                //try to re-center the box and x, y
                double dx = fabs(x - real_box.center().x);
                double dy = fabs(y - real_box.center().y);
                x = real_box.center().x;
                y = real_box.center().y;
                if(adjuster.adjust_edge_placement(agg::trans_affine(), box_elem.box_, detector_.extent(), real_box, x, y))
                {
                    real_box.re_center(x, y);
                    x += dx;
                    y += dy;
                }
            }
        }
        if (collision(real_box, box_elem.repeat_key_))
        {
            return false;
        }
        real_boxes.push_back(real_box);
    }

    // add boxes to collision detector
    std::list<box_element>::iterator elem_itr = box_elements_.begin();
    std::list< box2d<double> >::iterator real_itr = real_boxes.begin();
    while (elem_itr != box_elements_.end() && real_itr != real_boxes.end())
    {
        detector_.insert(*real_itr, elem_itr->repeat_key_);
        elem_itr++;
        real_itr++;
    }
    if(pos.x != x || pos.y != y)
    {
        results_.push_back(pixel_position(x,y));
    }
    else
    {
        results_.push_back(pos);
    }
    
    return true;
}

bool group_symbolizer_helper::collision(box2d<double> const& box, value_unicode_string const& repeat_key) const
{
    if (!detector_.extent().intersects(box)
            ||
        (text_props_->avoid_edges && !dims_.contains(box))
            ||
        (text_props_->minimum_padding > 0 &&
         !dims_.contains(box + (scale_factor_ * text_props_->minimum_padding)))
            ||
        (!text_props_->allow_overlap &&
            ((repeat_key.length() == 0 && !detector_.has_placement(box, text_props_->margin * scale_factor_))
                ||
             (repeat_key.length() > 0  && !detector_.has_placement(box, text_props_->margin * scale_factor_,
                                                                   repeat_key, text_props_->repeat_distance * scale_factor_))))
        )
    {
        return true;
    }
    return false;
}

double group_symbolizer_helper::get_spacing(double path_length) const
{
    int num_labels = 1;
    if (text_props_->label_spacing > 0)
    {
        num_labels = static_cast<int>(std::floor(
            path_length / (text_props_->label_spacing * scale_factor_)));
    }
    if (num_labels <= 0)
    {
        num_labels = 1;
    }
    return path_length / num_labels;
}

} //namespace
