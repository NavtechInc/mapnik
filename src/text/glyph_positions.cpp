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
//mapnik
#include <mapnik/text/glyph_positions.hpp>
#include <mapnik/pixel_position.hpp>
#include <mapnik/text/rotation.hpp>
#include <mapnik/text/glyph_info.hpp>

// stl
#include <vector>

namespace mapnik
{

glyph_positions::glyph_positions()
    : data_(),
      base_point_(),
      marker_(),
      marker_pos_(),
      marker_placement_tr_(),  //WI: Added John's changes for marker rotation 13NOV2014 (identity transform)
      bbox_() {}

glyph_positions::const_iterator glyph_positions::begin() const
{
    return data_.begin();
}

glyph_positions::const_iterator glyph_positions::end() const
{
    return data_.end();
}

void glyph_positions::emplace_back(glyph_info const& glyph, pixel_position offset, rotation const& rot)
{
    data_.emplace_back(glyph, offset, rot);
}

void glyph_positions::reserve(unsigned count)
{
    data_.reserve(count);
}

pixel_position const& glyph_positions::get_base_point() const
{
    return base_point_;
}

void glyph_positions::set_base_point(pixel_position const& base_point)
{
    base_point_ = base_point;
}

void glyph_positions::set_marker(marker_info_ptr marker, pixel_position const& marker_pos, box2d<double> const& marker_bbox,agg::trans_affine const& marker_placement_tr)
    {
    marker_ = marker;
    marker_pos_ = marker_pos;
        
    // GG (Feb 2015), recalculate the bounding box
    box2d<double>  bboxShield = marker_bbox;
    coord<double,2> c = bboxShield.center();
    bboxShield.re_center(0,0);
    bboxShield*=marker_placement_tr;//apply any special placement transform
    bboxShield.re_center(c);
    marker_bbox_ = bboxShield;
    //WI: Added John's changes for marker rotation 13NOV2014 (transform in general)
    marker_placement_tr_ *= marker_placement_tr;
}

marker_info_ptr glyph_positions::marker() const
{
    return marker_;
}

pixel_position const& glyph_positions::marker_pos() const
{
    return marker_pos_;
}

box2d<double> const & glyph_positions::marker_bbox() const
{
    return marker_bbox_;
}
}// ns mapnik
