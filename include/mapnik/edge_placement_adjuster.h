//
//  edge_placement_adjuster.h
//  mapnik-ios
//
//  Created by Walid Ibrahim on 2014-11-20.
//  Copyright (c) 2014 Navtech Inc. All rights reserved.
//

#ifndef mapnik_edge_placement_adjuster_h
#define mapnik_edge_placement_adjuster_h

//mapnik
#include <mapnik/box2d.hpp>
#include <mapnik/util/noncopyable.hpp>

//agg
#include "agg_trans_affine.h"

namespace mapnik
{
    class edge_placement_adjuster : util::noncopyable
    {
    public:
        inline bool adjust_edge_placement(agg::trans_affine const& tr, box2d<double> const& size, box2d<double> const& extent, box2d<double> const& box, double& x, double& y)
        {
            if(!extent.contains(box)) //WI: this will guarantee points falling outside buffer zone are not placed
            {
                return false;
            }
            
            box2d<double> tr_size = perform_transform(tr, size); //WI: apply scale to the size box to get the right width/height for the given scale
            double width_limit = tr_size.width() / 2.0;
            double height_limit = tr_size.height() / 2.0;
            
            /*
             *  the comparison below is because the holding are being relocated using a "bigger" bounding box. Whith this change the relocation is the minimun needed and the final efect is
             *  that the feature is redered closer to the real geographical location.
             */
            if (tr_size.width() > box.width() )
            {
                width_limit =box.width()/2.0;
                height_limit = box.height() / 2.0;
            }
            box2d<double> unbuffered_extent (extent.minx() - extent.minx(),
                                             extent.miny() - extent.miny(),
                                             extent.maxx() + extent.minx(),
                                             extent.maxy() + extent.miny());
 
            if(!unbuffered_extent.contains(box))
            {
                return adjust_x_y(unbuffered_extent, width_limit, height_limit, x, y);
            }
           /* else
            {
                box2d<double> placement_box = perform_transform(tr, box);
                bool centerd = (x == placement_box.center().x && y == placement_box.center().y);
                if(!centerd)
                {
                    if(!unbuffered_extent.contains(placement_box))
                    {
                        double placement_x, placement_y;
                        placement_x =  placement_box.center().x;
                        placement_y = placement_box.center().y ;
                        if(adjust_x_y(unbuffered_extent, width_limit, height_limit, placement_x, placement_y))
                        {
                            x = placement_x;
                            y = placement_y;
                            return true;
                        }
                        
                    }
                }
            }
            */
            return false;
        }
    private:
        inline bool adjust_x_y(const box2d<double>& unbuffered_extent, const double& width_limit, const double& height_limit, double& x, double& y)
        {
            bool adjusted = false;
            //the marker will be clipped try to find which x or y is clipped
            double eps =  0.0001;
            double padding = 1.0; //@TODO should be the stroke-width if set
            //move the x point
            if( std::abs( x - unbuffered_extent.minx() ) - width_limit < eps)
            {
                x = unbuffered_extent.minx() + width_limit + padding;
                adjusted = true;
            }
            if( std::abs( x - unbuffered_extent.maxx() ) - width_limit < eps )
            {
                x = unbuffered_extent.maxx() - width_limit - padding;
                adjusted = true;
            }
            //move the y point
            if( std::abs( y - unbuffered_extent.miny() ) - height_limit < eps )
            {
                y = unbuffered_extent.miny() + height_limit + padding;
                adjusted = true;
            }
            if( std::abs (  y - unbuffered_extent.maxy() ) - height_limit < eps )
            {
                y = unbuffered_extent.maxy() - height_limit - padding;
                adjusted = true;
            }
            return adjusted;
        }
        
        inline box2d<double> perform_transform(agg::trans_affine const& tr, box2d<double> const& size)
        {
            double x1 = size.minx();
            double x2 = size.maxx();
            double y1 = size.miny();
            double y2 = size.maxy();
            double xA = x1, yA = y1,
            xB = x2, yB = y1,
            xC = x2, yC = y2,
            xD = x1, yD = y2;
            tr.transform(&xA, &yA);
            tr.transform(&xB, &yB);
            tr.transform(&xC, &yC);
            tr.transform(&xD, &yD);
            box2d<double> result(xA, yA, xC, yC);
            result.expand_to_include(xB, yB);
            result.expand_to_include(xD, yD);
            return result;
        }
    };
}
#endif
