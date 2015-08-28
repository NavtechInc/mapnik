//
//  marker_placement_finder.cpp
//  mapnik-ios
//
//  Created by Walid Ibrahim on 2014-11-13.
//  Copyright (c) 2014 Navtech Inc. All rights reserved.
//

#include <mapnik/text/marker_placement_finder.hpp>

using  namespace mapnik;
marker_placement_finder::marker_placement_finder(text_layout_ptr const& layout_ptr)
    :glyph_bboxes_centre_line_(),
     glyph_angle_ccw_centre_line_(),
     layout_ptr_(layout_ptr)
{
}

marker_placement_finder::~marker_placement_finder()
{
}

void marker_placement_finder::line_size(int size)
{
    glyph_bboxes_centre_line_.reserve(size);
    glyph_angle_ccw_centre_line_.reserve(size);
}

void marker_placement_finder::add_angle(double angle)
{
    glyph_angle_ccw_centre_line_.push_back(angle);
}

void marker_placement_finder::add_bbox(box2d<double> const& bbox)
{
    glyph_bboxes_centre_line_.push_back(box2d<double>(bbox));
}

void marker_placement_finder::find_marker_placement(double scale_factor, pixel_position& marker_position, double& marker_cw_angle) const
{
    //Determine the area and orientation for the marker.
    //  1)  Collect bounding boxes and angles for glyphs in the centre line. (done above)
    //  2)  Find the glyphs closest to the centre of the line, and average their
    //      positions and angles to find where the marker should go.
    int nCentralBoundingBoxes = 0;
    marker_cw_angle = 0;
    coord<double,2> marker_coord(0, 0);
    
    if(glyph_bboxes_centre_line_.size() > 0)
    {
        int left_middle_glyph = ((glyph_bboxes_centre_line_.size()-1) / 2);
        int right_middle_glyph = (glyph_bboxes_centre_line_.size() / 2);
        for(int glyph = left_middle_glyph; glyph <= right_middle_glyph; ++glyph)
        {
            //Convert counter-clockwise glyph angle to clockwise marker
            //  angle, and accumulate for later averaging
            marker_cw_angle -= glyph_angle_ccw_centre_line_[glyph];
            
            //Accumulate the centre position of glyph bounding boxes for
            //  later averaging
            marker_coord = marker_coord + glyph_bboxes_centre_line_[glyph].center();
            
            nCentralBoundingBoxes++;
        }
    }
    
    //Average the angle and position to get the true position and angle of
    //  the marker.
    if(nCentralBoundingBoxes > 0)
    {
        marker_cw_angle /= (float)nCentralBoundingBoxes;
        marker_coord /= (float)nCentralBoundingBoxes;
    }
    
    //NOTE: line placement for shield will likely not work when no text glyphs exist
    //@TODO: find the middle when no glyphs exist (nCentralBoundingBoxes == 0)
    
    //WI: place the shield marker at the center of the marker box computed above
    marker_position.set(marker_coord.x,  marker_coord.y);
    pixel_position marker_displacement = scale_factor * layout_ptr_->displacement() + layout_ptr_->alignment_offset();
    if (layout_ptr_->rotate_displacement())
    {
        marker_displacement = marker_displacement.rotate(!layout_ptr_->orientation());
    }
    marker_position = marker_position + marker_displacement;
}