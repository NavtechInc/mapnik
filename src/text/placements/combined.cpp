/*****************************************************************************
 *
 * This file is part of Mapnik (c++ mapping toolkit)
 *
 * Copyright (C) 2015 Walid Ibrahim
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
#include <mapnik/debug.hpp>
#include <mapnik/text/placements/combined.hpp>
#include <mapnik/xml_node.hpp>

#include <mapnik/text/placements/list.hpp>
#include <mapnik/text/placements/simple.hpp>

namespace mapnik
{

//text_placements_combined class
text_placements_combined::text_placements_combined(text_placements_ptr simple_placement, text_placements_ptr list_placement)
    :simple_placement_(simple_placement),
    list_placement_(list_placement),
    reversed_(false)
{
}
    
text_placements_combined::text_placements_combined(text_placements_ptr simple_placement, text_placements_ptr list_placement, bool reverse)
    :simple_placement_(simple_placement),
     list_placement_(list_placement),
     reversed_(reverse)
{
}

text_placement_info_ptr text_placements_combined::get_placement_info(double scale_factor, feature_impl const& feature, attributes const& vars) const
{
    text_placement_info_ptr simple_placement_info = simple_placement_->get_placement_info(scale_factor, feature, vars);
    
    text_placement_info_ptr list_placement_info = list_placement_->get_placement_info(scale_factor, feature, vars);
    
    return std::make_shared<text_placement_info_combined>(this, scale_factor, simple_placement_info, list_placement_info, reversed_);
}
    
void text_placements_combined::add_expressions(expression_set & output) const
{
    simple_placement_->add_expressions(output);
    list_placement_->add_expressions(output);
}

text_placements_ptr text_placements_combined::from_xml(xml_node const& xml, fontset_map const& fontsets, bool is_shield)
{
    text_placements_ptr simple_placement_ptr = text_placements_simple::from_xml(xml, fontsets, is_shield);
    text_placements_ptr list_placement_ptr = text_placements_list::from_xml(xml, fontsets, is_shield);
    if(simple_placement_ptr && list_placement_ptr)
    {
        text_placements_ptr ptr = std::make_shared<text_placements_combined>(simple_placement_ptr, list_placement_ptr);
        ptr->defaults.from_xml(xml, fontsets, is_shield);
        return ptr;
    }
    return text_placements_ptr();
}
    
//text_placements_reversed_combined class
text_placements_reverse_combined::text_placements_reverse_combined(text_placements_ptr simple_placement, text_placements_ptr list_placement)
    :text_placements_combined(simple_placement, list_placement, true)
{
}
    
text_placements_ptr text_placements_reverse_combined::from_xml(xml_node const& xml, fontset_map const& fontsets, bool is_shield)
{
    text_placements_ptr simple_placement_ptr = text_placements_simple::from_xml(xml, fontsets, is_shield);
    text_placements_ptr list_placement_ptr = text_placements_list::from_xml(xml, fontsets, is_shield);
    if(simple_placement_ptr && list_placement_ptr)
    {
        text_placements_ptr ptr = std::make_shared<text_placements_reverse_combined>(simple_placement_ptr, list_placement_ptr);
        ptr->defaults.from_xml(xml, fontsets, is_shield);
        return ptr;
    }
    return text_placements_ptr();
}

//text_placement_info_combined class
text_placement_info_combined::text_placement_info_combined(text_placements_combined const* parent, double scale_factor, text_placement_info_ptr simple_placement_info, text_placement_info_ptr list_placement_info, bool reversed)
: text_placement_info(parent, scale_factor),
  simple_placement_info_(simple_placement_info),
  list_placement_info_(list_placement_info),
  reversed_(reversed)
{
}
    
void text_placement_info_combined::reset_state()
{
    simple_placement_info_->reset_state();
    list_placement_info_->reset_state();
}
    
bool text_placement_info_combined::next() const
{
    //logic to get the next combined point placement
    if(simple_placement_info_ && list_placement_info_)
    {
      if(reversed_)
      {
          //apply list first then simple
          return apply_placements(list_placement_info_, simple_placement_info_);
      }
      else
      {
          //apply simple first then list
          return apply_placements(simple_placement_info_, list_placement_info_);
      }
    }
    return false;
}
    
bool text_placement_info_combined::apply_placements(text_placement_info_ptr first, text_placement_info_ptr second) const
{
    //try the first placement options
    if(first->next())
    {
        apply_placement(first);
    }
    else if(second->next()) //try the second placement options
    {
        //first placement needs to reset state
        first->reset_state();
        first->properties = second->properties;
        apply_placement(second);
        if(first->next()) //re-apply the first placement
        {
            apply_placement(first);
        }
    }
    else
    {
        return false;
    }
    return true;
}
    
void text_placement_info_combined::apply_placement(text_placement_info_ptr placement_info) const
{
    text_placement_info_simple* simple = dynamic_cast<text_placement_info_simple*>(placement_info.get());
    if (simple)
    {
        apply_simple_placement();
    }
    else
    {
        text_placement_info_list* list = dynamic_cast<text_placement_info_list*>(placement_info.get());
        if (list)
        {
            apply_list_placement();
        }
        else
        {
              MAPNIK_LOG_ERROR(text_placement_info_combined) << "Could not apply unknown placement info ptr";
        }
    }
}

void text_placement_info_combined::apply_list_placement() const
{
    properties = list_placement_info_->properties;
}
    
void text_placement_info_combined::apply_simple_placement() const
{
    properties.format_defaults.text_size = simple_placement_info_->properties.format_defaults.text_size;
    properties.layout_defaults.dir = simple_placement_info_->properties.layout_defaults.dir;
}
    
}//namespace