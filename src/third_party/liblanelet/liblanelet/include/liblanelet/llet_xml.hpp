/*
 *  © 2014 by Philipp Bender <pbender@fzi.de>
 * 
 *  This file is part of libLanelet.
 *
 *  libLanelet is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  libLanelet is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with libLanelet.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <vector>
#include "liblanelet/Lanelet.hpp"
#include "liblanelet/LaneletPoint.hpp"

namespace LLet
{
typedef std::vector<point_with_id_t> reference_line_t;

struct LaneletXML
{
    std::vector<lanelet_ptr_t> lanelets;
    bool local_origin_defined = false;
    point_with_id_t reference_origin;
    reference_line_t drivable_boundary;
};

std::vector<lanelet_ptr_t> parse_xml(std::string filename);
LaneletXML parse_xml_full(std::string filename);
}
