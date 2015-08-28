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
#include <mapnik/text/renderer.hpp>
#include <mapnik/graphics.hpp>
#include <mapnik/grid/grid.hpp>
#include <mapnik/text/text_properties.hpp>
#include <mapnik/font_engine_freetype.hpp>
#include <mapnik/text/face.hpp>

namespace mapnik
{

text_renderer::text_renderer (halo_rasterizer_e rasterizer, composite_mode_e comp_op,
                              composite_mode_e halo_comp_op, double scale_factor, stroker_ptr stroker)
    : rasterizer_(rasterizer),
      comp_op_(comp_op),
      halo_comp_op_(halo_comp_op),
      scale_factor_(scale_factor),
      glyphs_(),
      stroker_(stroker),
      transform_(),
      halo_transform_()
{}

void text_renderer::set_transform(agg::trans_affine const& transform)
{
    transform_ = transform;
}

void text_renderer::set_halo_transform(agg::trans_affine const& halo_transform)
{
    halo_transform_ = halo_transform;
}

void text_renderer::prepare_glyphs(glyph_positions const& positions)
{
    FT_Matrix matrix;
    FT_Vector pen;
    FT_Error  error;

    glyphs_.reserve(positions.size());
    for (auto const& glyph_pos : positions)
    {
        glyph_info const& glyph = glyph_pos.glyph;
        glyph.face->set_character_sizes(glyph.format->text_size * scale_factor_); //TODO: Optimize this?

        matrix.xx = static_cast<FT_Fixed>( glyph_pos.rot.cos * 0x10000L);
        matrix.xy = static_cast<FT_Fixed>(-glyph_pos.rot.sin * 0x10000L);
        matrix.yx = static_cast<FT_Fixed>( glyph_pos.rot.sin * 0x10000L);
        matrix.yy = static_cast<FT_Fixed>( glyph_pos.rot.cos * 0x10000L);

        pixel_position pos = glyph_pos.pos + glyph.offset.rotate(glyph_pos.rot);
        pen.x = static_cast<FT_Pos>(pos.x * 64);
        pen.y = static_cast<FT_Pos>(pos.y * 64);

        FT_Face face = glyph.face->get_face();
        FT_Set_Transform(face, &matrix, &pen);

        error = FT_Load_Glyph(face, glyph.glyph_index, FT_LOAD_NO_HINTING);
        if (error) continue;

        FT_Glyph image;
        error = FT_Get_Glyph(face->glyph, &image);
        if (error) continue;

        glyphs_.emplace_back(image, *glyph.format);
    }
}
    
// remove shield background
template <typename T>
void composite_bitmap_remove_background(T & pixmap, box2d<double> const& box)
{
    int x_max=box.maxx();
    int y_max=box.maxy();
    int i,p,j,q;
    color black(0,0,0);
    unsigned fillColor =black.rgba();

    for (i=box.minx(),p=0;i<x_max;++i,++p)
    {
        for (j=box.miny(),q=0;j<y_max;++j,++q)
        {
            pixmap.composite_pixel(dst_out, i, j, fillColor, fillColor, 1);//dst_out
        }
    }

}
    
template <typename T>
void composite_bitmap(T & pixmap, FT_Bitmap *bitmap, unsigned rgba, int x, int y, double opacity, composite_mode_e comp_op)
{
    int x_max=x+bitmap->width;
    int y_max=y+bitmap->rows;
    int i,p,j,q;

    for (i=x,p=0;i<x_max;++i,++p)
    {
        for (j=y,q=0;j<y_max;++j,++q)
        {
            unsigned gray=bitmap->buffer[q*bitmap->width+p];
            if (gray)
            {
                pixmap.composite_pixel(comp_op, i, j, rgba, gray, opacity);
            }
        }
    }
}

template <typename T>
agg_text_renderer<T>::agg_text_renderer (pixmap_type & pixmap,
                                         halo_rasterizer_e rasterizer,
                                         composite_mode_e comp_op,
                                         composite_mode_e halo_comp_op,
                                         double scale_factor,
                                         stroker_ptr stroker)
    : text_renderer(rasterizer, comp_op, halo_comp_op, scale_factor, stroker), pixmap_(pixmap)
{}

long distance (pixel_position p1, pixel_position p2)
{
    long dx = (p2.x-p1.x);
    long dy = (p2.y-p1.y);
    
    return dx*dx + dy*dy;
}

void draw_leading_line (image_32 &pixmap, box2d<double> const& box, pixel_position const& base_point,unsigned color, float scale)
{
    
    /*
    
                (x0,y0)       (x1,y0)      (x2,y0)
                       --------------------
                      |                    |
                      |                    |
            (x0,y1)   |                    |  (x2,y1)
                      |                    |
                      |                    |
                       --------------------
                (x0,y2)       (x1,y2)      (x2,y2)
    */
    int padding =1*scale;
    int x0 = static_cast<int>(box.minx())-padding;
    int x2 = static_cast<int>(box.maxx())+padding;
    int y0 = static_cast<int>(box.miny())-padding;
    int y2 = static_cast<int>(box.maxy())+padding;
    
    pixel_position originalPoint(int(base_point.x), int(base_point.y));
    if (x0<0) x0 =0;
    if (y0<0) y0 =0;
    if (x2>pixmap.width()) x2 =pixmap.width();
    if (y2>pixmap.height()) y2 =pixmap.height();
    
    int x1 = (x0 + x2) /2;
    int y1 = (y0 + y2) /2;
    
    // leading line
    pixel_position corner1(x0, y0);
    pixel_position corner2(x2, y0);
    pixel_position corner3(x0, y2);
    pixel_position corner4(x2, y2);

    pixel_position midPoint1(x1, y0);     // midle of the top side
    pixel_position midPoint2(x1, y2);     //  midle of the lower side
    pixel_position midPoint3(x0, y1);     //  midle of the left side
    pixel_position midPoint4(x2, y1);     //  midle of the right side
    
    pixel_position pointArray[]={corner1, corner2,corner3,corner4,midPoint1,midPoint2,midPoint3,midPoint4};
   
    pixel_position corner = corner1;
    long dis =100000;
    long newDis;

    for ( int i =0, numberOfPoints = 8; i< numberOfPoints;i++)
    {
        pixel_position newCorner = pointArray[i];
        newDis = distance(originalPoint, newCorner);
        if (dis> newDis)
        {
            dis = newDis;
            corner = newCorner;
        }
    }

    
    //if (sqrt(dis) > (y2-y0) /2 )      // only draw the leading line if the distance is bigger than height/2 or it can be defined using pixels and scale.
    {
        pixel_position pointFrom;
        pixel_position pointTo;
        if (corner.x < originalPoint.x)
        {
            pointFrom = corner;
            pointTo = originalPoint;
        }
        else if (corner.x > originalPoint.x)
        {
            pointFrom = originalPoint;
            pointTo = corner;
        }
        else if (corner.y > originalPoint.y)
        {
            pointFrom = originalPoint;
            pointTo = corner;
        }
        else
        {
            pointFrom = corner;
            pointTo = originalPoint;
        }
        /*  drawing the line
                y = mx + b
            where:
        
        m   is the slope or gradient of the line.
        b   is the y-intercept of the line.
        x   is the independent variable of the function y = f(x).
        
         */
        
        float m = 0;
        
        if (pointTo.x != pointFrom.x )  // it isn't a vertical line
        {
            m= (pointTo.y - pointFrom.y) /( pointTo.x - pointFrom.x);
        
            float b = pointTo.y - m*pointTo.x;
            for (int posX = pointFrom.x; posX<pointTo.x ; ++posX)
            {
                int posY = m * posX + b;
                
                pixmap.setPixel(posX, posY, color);
                if (scale == 2 )
                {
                    pixmap.setPixel(posX+1, posY, color);
                }
                int posY1 = m * (posX+1) + b;
                if ( posY1 != posY )
                {
                    int sign = (posY - posY1) /abs(posY1-posY);
                    
                    while (abs(posY1-posY)>1)           //  when calculating Y for X and X+1 and the difference in greater than 1,
                                                        //  we want to keep drawing the line on the Y axe to avoid a "jump" visual efect .
                    {
                        posY = posY- sign;
                        pixmap.setPixel(posX, posY, color);
                        if (scale == 2 )        // the line is thicker (retina display devices)
                        {
                            pixmap.setPixel(posX+1, posY, color);
                        }
                    }
                }
                
            }
        }
        else    // It's a vertical line (m = 0)
        {
            for (int posY = pointFrom.y; posY<pointTo.y ; ++posY)
            {
                pixmap.setPixel(pointFrom.x, posY, color);
                if (scale == 2 )
                {
                    pixmap.setPixel(pointFrom.x+1, posY, color);
                }
            }
        }
    }
}

    /*
     this function was modified to:
        1. draw a leading line to the text (a bounding box is calculated)
        2. remove the background when drawing the characters/marker.
        3. receive an addiontal parameter used to as an extreme for the leading line (original point).
     Gerardo Granados, Feb 2015.
     */
template <typename T>
void agg_text_renderer<T>::render(glyph_positions const& pos, pixel_position original_point)
{
    glyphs_.clear();
    prepare_glyphs(pos);
    FT_Error  error;
    FT_Vector start;
    FT_Vector start_halo;
    int height = pixmap_.height();
    pixel_position const& base_point = pos.get_base_point();
    
    start.x =  static_cast<FT_Pos>(base_point.x * (1 << 6));
    start.y =  static_cast<FT_Pos>((height - base_point.y) * (1 << 6));
    start_halo = start;
    start.x += transform_.tx * 64;
    start.y += transform_.ty * 64;
    start_halo.x += halo_transform_.tx * 64;
    start_halo.y += halo_transform_.ty * 64;
    
    FT_Matrix halo_matrix;
    halo_matrix.xx = halo_transform_.sx  * 0x10000L;
    halo_matrix.xy = halo_transform_.shx * 0x10000L;
    halo_matrix.yy = halo_transform_.sy  * 0x10000L;
    halo_matrix.yx = halo_transform_.shy * 0x10000L;
    
    FT_Matrix matrix;
    matrix.xx = transform_.sx  * 0x10000L;
    matrix.xy = transform_.shx * 0x10000L;
    matrix.yy = transform_.sy  * 0x10000L;
    matrix.yx = transform_.shy * 0x10000L;
    
    // default formatting
    double halo_radius = 0;
    color black(0,0,0);
    unsigned fill = black.rgba();
    unsigned halo_fill = black.rgba();
    double text_opacity = 1.0;
    double halo_opacity = 1.0;
    
    unsigned rgbaLeadingLineColor;
    bool     drawLeadingLine = false;
    bool     maskBackground = false;
    
    if ( glyphs_.size() > 0 )
    {
        glyph_t glyph = glyphs_.front();
        rgbaLeadingLineColor = glyph.properties.fill.rgba();
        if (glyph.properties.leading_line && original_point.length() >0 )
        {
            drawLeadingLine = true;
        }
    }

    bool masked_marker = false;
    // remove the background for each glyph
    for (auto const& glyph : glyphs_)
    {
        maskBackground =glyph.properties.mask_background;
        if(!maskBackground) continue;
        if (maskBackground && pos.marker() && !masked_marker)   // remove the marker background
        {
            box2d<double>  bboxShield = pos.marker_bbox();
            composite_bitmap_remove_background(pixmap_,bboxShield);
            masked_marker = true;
        }
        double stroke =2* scale_factor_;            // using the halo_radious to draw the letters and calculate the bounding box.
        if (glyph.properties.halo_radius>2)
        {
            stroke =glyph.properties.halo_radius * scale_factor_;
        }

        FT_Glyph g;
        error = FT_Glyph_Copy(glyph.image, &g);
        if (!error)
        {
            FT_Glyph_Transform(g, &matrix, &start);

                stroker_->init(stroke);
                FT_Glyph_Stroke(&g, stroker_->get(), 1);
                error = FT_Glyph_To_Bitmap(&g, FT_RENDER_MODE_NORMAL, 0, 1);
                if (!error)
                {
                    FT_BBox glyph_bbox;
                    FT_Glyph_Get_CBox(g, FT_GLYPH_BBOX_PIXELS, &glyph_bbox);
                    box2d<double> bounding_box =box2d<double>(glyph_bbox.xMin, height-glyph_bbox.yMin,glyph_bbox.xMax, height-glyph_bbox.yMax);
                    composite_bitmap_remove_background(pixmap_,bounding_box);
                }
        }
        
        FT_Done_Glyph(g);
    }
    
    // halo rendering
    for (auto const& glyph : glyphs_)
    {
        halo_fill = glyph.properties.halo_fill.rgba();
        halo_opacity = glyph.properties.halo_opacity;
        halo_radius = glyph.properties.halo_radius * scale_factor_;
        // make sure we've got reasonable values.
        if (halo_radius <= 0.0 || halo_radius > 1024.0) continue;
        FT_Glyph g;
        error = FT_Glyph_Copy(glyph.image, &g);
        if (!error)
        {
            FT_Glyph_Transform(g, &halo_matrix, &start_halo);
            if (rasterizer_ == HALO_RASTERIZER_FULL)
            {
                stroker_->init(halo_radius);
                FT_Glyph_Stroke(&g, stroker_->get(), 1);
                error = FT_Glyph_To_Bitmap(&g, FT_RENDER_MODE_NORMAL, 0, 1);
                if (!error)
                {
                    FT_BitmapGlyph bit = reinterpret_cast<FT_BitmapGlyph>(g);
                    composite_bitmap(pixmap_,
                                     &bit->bitmap,
                                     halo_fill,
                                     bit->left,
                                     height - bit->top,
                                     halo_opacity,
                                     halo_comp_op_);
                }
            }
            else
            {
                error = FT_Glyph_To_Bitmap(&g, FT_RENDER_MODE_NORMAL, 0, 1);
                if (!error)
                {
                    FT_BitmapGlyph bit = reinterpret_cast<FT_BitmapGlyph>(g);
                    render_halo(&bit->bitmap,
                                halo_fill,
                                bit->left,
                                height - bit->top,
                                halo_radius,
                                halo_opacity,
                                halo_comp_op_);
                }
            }
        }
        FT_Done_Glyph(g);
    }
    
    box2d<double> bounding_box;     // to get the bounding box around the text
    box2d<double> empty_box;

    // render actual text
    for (auto & glyph : glyphs_)
    {
        fill = glyph.properties.fill.rgba();
        text_opacity = glyph.properties.text_opacity;
        FT_Glyph_Transform(glyph.image, &matrix, &start);
        error = FT_Glyph_To_Bitmap(&glyph.image ,FT_RENDER_MODE_NORMAL, 0, 1);
        if (!error)
        {
            FT_BitmapGlyph bit = reinterpret_cast<FT_BitmapGlyph>(glyph.image);
            
            int x= bit->left;
            int y =height - bit->top;
            composite_bitmap(pixmap_,
                             &bit->bitmap,
                             fill,
                             bit->left,
                             height - bit->top,
                             text_opacity,
                             comp_op_);

            if (bit->bitmap.rows>0 && drawLeadingLine)
            {
                box2d<double> nextBox = box2d<double>(x,y,x+bit->bitmap.width,y+bit->bitmap.rows);
                if (bounding_box == empty_box )
                {
                    bounding_box =nextBox;
                }
                else
                {
                    bounding_box.expand_to_include(nextBox );
                }
            }
        }
        
        FT_Done_Glyph(glyph.image);
    }

    if (drawLeadingLine)
    {
        // if there is a marker the bounding box may need to be expanded
        if (pos.marker())
        {
            box2d<double> const& bboxMarker = pos.marker_bbox();
            bounding_box.expand_to_include(bboxMarker);
        }
        draw_leading_line(pixmap_, bounding_box, original_point,rgbaLeadingLineColor, scale_factor_);
    }
}

template <typename T>
void grid_text_renderer<T>::render(glyph_positions const& pos, value_integer feature_id)
{
    glyphs_.clear();
    prepare_glyphs(pos);
    FT_Error  error;
    FT_Vector start;
    unsigned height = pixmap_.height();
    pixel_position const& base_point = pos.get_base_point();
    start.x =  static_cast<FT_Pos>(base_point.x * (1 << 6));
    start.y =  static_cast<FT_Pos>((height - base_point.y) * (1 << 6));
    start.x += transform_.tx * 64;
    start.y += transform_.ty * 64;

    // now render transformed glyphs
    double halo_radius = 0.0;
    FT_Matrix halo_matrix;
    halo_matrix.xx = halo_transform_.sx  * 0x10000L;
    halo_matrix.xy = halo_transform_.shx * 0x10000L;
    halo_matrix.yy = halo_transform_.sy  * 0x10000L;
    halo_matrix.yx = halo_transform_.shy * 0x10000L;
    for (auto & glyph : glyphs_)
    {
        halo_radius = glyph.properties.halo_radius * scale_factor_;
        FT_Glyph_Transform(glyph.image, &halo_matrix, &start);
        error = FT_Glyph_To_Bitmap(&glyph.image, FT_RENDER_MODE_NORMAL, 0, 1);
        if (!error)
        {

            FT_BitmapGlyph bit = reinterpret_cast<FT_BitmapGlyph>(glyph.image);
            render_halo_id(&bit->bitmap,
                           feature_id,
                           bit->left,
                           height - bit->top,
                           static_cast<int>(halo_radius));
        }
        FT_Done_Glyph(glyph.image);
    }
}


template <typename T>
void agg_text_renderer<T>::render_halo(FT_Bitmap *bitmap,
                 unsigned rgba,
                 int x1,
                 int y1,
                 double halo_radius,
                 double opacity,
                 composite_mode_e comp_op)
{
    int width = bitmap->width;
    int height = bitmap->rows;
    int x, y;
    if (halo_radius < 1.0)
    {
        for (x=0; x < width; x++)
        {
            for (y=0; y < height; y++)
            {
                int gray = bitmap->buffer[y*bitmap->width+x];
                if (gray)
                {
                    pixmap_.composite_pixel(comp_op, x+x1-1, y+y1-1, rgba, gray*halo_radius*halo_radius, opacity);
                    pixmap_.composite_pixel(comp_op, x+x1,   y+y1-1, rgba, gray*halo_radius, opacity);
                    pixmap_.composite_pixel(comp_op, x+x1+1, y+y1-1, rgba, gray*halo_radius*halo_radius, opacity);

                    pixmap_.composite_pixel(comp_op, x+x1-1, y+y1,   rgba, gray*halo_radius, opacity);
                    pixmap_.composite_pixel(comp_op, x+x1,   y+y1,   rgba, gray, opacity);
                    pixmap_.composite_pixel(comp_op, x+x1+1, y+y1,   rgba, gray*halo_radius, opacity);

                    pixmap_.composite_pixel(comp_op, x+x1-1, y+y1+1, rgba, gray*halo_radius*halo_radius, opacity);
                    pixmap_.composite_pixel(comp_op, x+x1,   y+y1+1, rgba, gray*halo_radius, opacity);
                    pixmap_.composite_pixel(comp_op, x+x1+1, y+y1+1, rgba, gray*halo_radius*halo_radius, opacity);
                }
            }
        }
    }
    else
    {
        for (x=0; x < width; x++)
        {
            for (y=0; y < height; y++)
            {
                int gray = bitmap->buffer[y*bitmap->width+x];
                if (gray)
                {
                    for (int n=-halo_radius; n <=halo_radius; ++n)
                        for (int m=-halo_radius; m <= halo_radius; ++m)
                            pixmap_.composite_pixel(comp_op, x+x1+m, y+y1+n, rgba, gray, opacity);
                }
            }
        }
    }
}

template <typename T>
void grid_text_renderer<T>::render_halo_id(
                    FT_Bitmap *bitmap,
                    mapnik::value_integer feature_id,
                    int x1,
                    int y1,
                    int halo_radius)
{
    int width = bitmap->width;
    int height = bitmap->rows;
    int x, y;
    for (x=0; x < width; x++)
    {
        for (y=0; y < height; y++)
        {
            int gray = bitmap->buffer[y*bitmap->width+x];
            if (gray)
            {
                for (int n=-halo_radius; n <=halo_radius; ++n)
                    for (int m=-halo_radius; m <= halo_radius; ++m)
                        pixmap_.setPixel(x+x1+m,y+y1+n,feature_id);
            }
        }
    }
}

template <typename T>
grid_text_renderer<T>::grid_text_renderer(pixmap_type &pixmap,
                                          composite_mode_e comp_op,
                                          double scale_factor)
    : text_renderer(HALO_RASTERIZER_FAST, comp_op, src_over, scale_factor),
      pixmap_(pixmap) {}

template class agg_text_renderer<image_32>;
template class grid_text_renderer<grid>;

} // namespace mapnik
