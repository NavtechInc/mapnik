//
//  marker_placement_finder.hpp
//  mapnik-ios
//
//  Created by Walid Ibrahim on 2014-11-13.
//  Copyright (c) 2014 Navtech Inc. All rights reserved.
//

#ifndef mapnik_ios_marker_placement_finder_hpp
#define mapnik_ios_marker_placement_finder_hpp

//mapnik
#include <mapnik/box2d.hpp>
#include <mapnik/pixel_position.hpp>

#include <mapnik/text/text_layout.hpp>
//stl
#include <vector>
#include <memory>

namespace mapnik
{
    class marker_placement_finder
    {
    public:
        marker_placement_finder(text_layout_ptr const& layout_ptr);
        virtual ~marker_placement_finder();
        
        void line_size(int size);
        void add_angle(double angle);
        void add_bbox(box2d<double> const& bbox);
        void align_offset( pixel_position const& align_offset ) { align_offset_ = align_offset; }
        void find_marker_placement(double scale_factor, pixel_position& marker_position, double& marker_cw_angle) const;
    private:
        //Track all bounding boxes by line, and all angles associated with glyphs on
        //  each part of the line, so that the marker can be accurately placed on
        //  the centre glyph.
        //Only track the line in the very centre to save time and memory
        //NOTE: only used if has_marker_ is true.
        std::vector<box2d<double> > glyph_bboxes_centre_line_;
        std::vector<double> glyph_angle_ccw_centre_line_;
        text_layout_ptr layout_ptr_;
        pixel_position align_offset_;
    };
}

using marker_placement_finder_ptr = std::shared_ptr<mapnik::marker_placement_finder>;
#endif
