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

#include "liblanelet/llet_xml.hpp"
#include "liblanelet/LineStrip.hpp"
#include "liblanelet/RegulatoryElement.hpp"

#include <libpugixml/pugixml.hpp>
#include <common/prettyprint.hpp>
#include <common/mercator.hpp>

#include <boost/format.hpp>

#include <map>
#include <iostream>
#include <cmath>
#include <cassert>

#define LLET_NAME corridor
#define REG_ELEM_NAME traffic_element

using namespace LLet;

namespace
{

struct TagWalker : public pugi::xml_tree_walker
{
    AttributeMap attributes;

    virtual bool for_each(pugi::xml_node &node)
    {
        if (node.name() == pugi::string_t("tag"))
        {
            std::string k = node.attribute("k").value();
            std::string v = node.attribute("v").value();
            attributes[k] = v;
        }

        return true;
    }
};

struct WayTreeWalker : public pugi::xml_tree_walker
{
    WayTreeWalker(std::map<int32_t, point_with_id_t> &p_b_i, OSMLineStrip &strp) : points_by_id(p_b_i), linestrip(strp)
    {
    }

    std::map<int32_t, point_with_id_t> &points_by_id;
    OSMLineStrip &linestrip;

    virtual bool for_each(pugi::xml_node &node)
    {
        if (node.name() == pugi::string_t("tag"))
        {
            std::string k = node.attribute("k").value();
            std::string v = node.attribute("v").value();
            linestrip._attributes[k] = v;
        }

        else if (node.name() == pugi::string_t("nd"))
        {
            int32_t node_id = boost::lexical_cast<int32_t>(node.attribute("ref").value());
            linestrip._pts.push_back(points_by_id[node_id]);
        }

        else
        {
            std::cout << node.name() << std::endl;
            std::cout << "doing nothing" << std::endl;
        }

        return true;
    }
};

struct XMLParser
{
    std::map<int32_t, point_with_id_t> points_by_id;
    std::map<int32_t, strip_ptr_t> linestrips_by_id;
    std::map<int32_t, lanelet_ptr_t> lanelets_by_id;
    std::map<int32_t, regulatory_element_ptr_t> regulatory_elements_by_id;

    // additonal information for using map in a local region
    bool local_origin_defined = false;
    point_with_id_t reference_origin;
    reference_line_t drivable_boundary;
    std::unordered_map<std::string, reference_line_t> center_lines;

    pugi::xml_document &doc;

    void parse_nodes();
    void parse_linestrips();
    void parse_lanelets();
    void parse_regulatory_elements_and_assign_to_lanelets();
    void parse_reference_points();
    void parse_center_lines();

    std::vector<lanelet_ptr_t> parsed_lanelets() const;
    bool parsed_reference_origin(point_with_id_t *pt) const;
    reference_line_t parsed_boundary_points() const;
    std::unordered_map<std::string, reference_line_t> parsed_center_lines() const;

    XMLParser(pugi::xml_document &doc) : doc(doc)
    {
        parse_nodes();
        parse_linestrips();
        parse_lanelets();
        parse_regulatory_elements_and_assign_to_lanelets();
        parse_reference_points();
        parse_center_lines();
    }
};

std::vector<lanelet_ptr_t> XMLParser::parsed_lanelets() const
{
    std::vector<lanelet_ptr_t> result;
    for (auto &kv : lanelets_by_id)
    {
        result.push_back(kv.second);
    }

    return result;
}

bool XMLParser::parsed_reference_origin(point_with_id_t *pt) const
{
    *pt = reference_origin;
    return local_origin_defined;
}

reference_line_t XMLParser::parsed_boundary_points() const
{
    return drivable_boundary;
}

std::unordered_map<std::string, reference_line_t> XMLParser::parsed_center_lines() const
{
    return center_lines;
}

void XMLParser::parse_nodes()
{
    for (pugi::xpath_node node : doc.select_nodes("//node[@lat and @lon and @id]"))
    {
        double lat = node.node().attribute("lat").as_double();
        double lon = node.node().attribute("lon").as_double();
        double id = node.node().attribute("id").as_int();
        points_by_id[id] = std::make_tuple(lat, lon, id);
    }
}

void XMLParser::parse_linestrips()
{
    for (pugi::xpath_node way : doc.select_nodes("//way[@id]"))
    {
        std::shared_ptr<OSMLineStrip> new_linestrip(new OSMLineStrip());
        WayTreeWalker walker(points_by_id, *new_linestrip);
        way.node().traverse(walker);
        linestrips_by_id[way.node().attribute("id").as_int()] = new_linestrip;
    }
}

void XMLParser::parse_lanelets()
{
    for (pugi::xpath_node relation : doc.select_nodes("//relation/tag[@v='lanelet' and @k='type']/.."))
    {
        std::map<std::string, strip_ptr_t> bounds;

        for (std::string role : {"left", "right"})
        {
            std::vector<strip_ptr_t> line_strips_for_this_bound;
            for (pugi::xpath_node member : relation.node().select_nodes((boost::format("member[@type='way' and @role='%s']") % role).str().c_str()))
            {
                int32_t id = member.node().attribute("ref").as_int();
                line_strips_for_this_bound.push_back(linestrips_by_id[id]);
            }

            CompoundLineStrip *comp = new CompoundLineStrip(line_strips_for_this_bound);

            bounds[role] = std::shared_ptr<LineStrip>(comp);
        }

        int32_t id = relation.node().attribute("id").as_int();

        TagWalker tagwalker;
        relation.node().traverse(tagwalker);

        const auto &pts_left = bounds["left"]->pts();
        const auto &pts_right = bounds["right"]->pts();

        if (dist(pts_left.front(), pts_right.front()) > dist(pts_left.front(), pts_right.back()))
        {
            bounds["left"] = std::make_shared<ReversedLineStrip>(bounds["left"]);
        }

        // angle between (a, b) should be somewhere near +pi/2.
        auto phi = [&bounds]() -> double {
            auto vec_b = vec(bounds["left"]->pts()[0], bounds["left"]->pts()[1]);
            auto vec_a = vec(bounds["left"]->pts()[0], bounds["right"]->pts()[0]);
            return angle(vec_a, vec_b);
        };

        //  std::cout << "angle is " << phi * 180 / M_PI << std::endl;
        double pi_2 = M_PI * 0.5;

        if (!inrange(phi(), 0.5 * pi_2, 1.5 * pi_2))
        {
            bounds["left"] = std::make_shared<ReversedLineStrip>(bounds["left"]);
            bounds["right"] = std::make_shared<ReversedLineStrip>(bounds["right"]);
        }

        assert(inrange(phi(), 0.5 * pi_2, 1.5 * pi_2));

        auto new_lanelet = std::make_shared<Lanelet>(id, bounds["left"], bounds["right"]);
        new_lanelet->attributes() = tagwalker.attributes;
        lanelets_by_id[id] = new_lanelet;
    }
}

void XMLParser::parse_regulatory_elements_and_assign_to_lanelets()
{
    for (pugi::xpath_node relation : doc.select_nodes("//relation/tag[@v='regulatory_element' and @k='type']/.."))
    {
        int32_t id = relation.node().attribute("id").as_int();

        TagWalker tagwalker;
        relation.node().traverse(tagwalker);

        auto new_reg_elem = std::make_shared<RegulatoryElement>(id);
        new_reg_elem->attributes() = tagwalker.attributes;

        std::vector<regulatory_element_member_t> all_members;

        for (pugi::xpath_node member : relation.node().select_nodes("member[@type='way']"))
        {
            int32_t ref = member.node().attribute("ref").as_int();
            std::string role = member.node().attribute("role").value();

            member_variant_t __member = linestrips_by_id[ref];
            assert(boost::get<strip_ptr_t>(__member) != nullptr);

            all_members.push_back(std::make_pair(role, __member));
        }

        for (pugi::xpath_node member : relation.node().select_nodes("member[@type='node']"))
        {
            int32_t ref = member.node().attribute("ref").as_int();
            std::string role = member.node().attribute("role").value();

            member_variant_t __member = points_by_id[ref];
            // not possible to check here for nulpointer ?
            all_members.push_back(std::make_pair(role, __member));
        }

        for (pugi::xpath_node member : relation.node().select_nodes("member[@type='relation']"))
        {
            int32_t ref = member.node().attribute("ref").as_int();
            std::string role = member.node().attribute("role").value();

            member_variant_t __member = lanelets_by_id[ref];
            assert(boost::get<lanelet_ptr_t>(__member) != nullptr);

            all_members.push_back(std::make_pair(role, __member));
        }

        new_reg_elem->members() = all_members;

        regulatory_elements_by_id[id] = new_reg_elem;
    }

    for (pugi::xpath_node lanelet : doc.select_nodes("//relation/tag[@v='lanelet' and @k='type']/.."))
    {
        int32_t relation_id = lanelet.node().attribute("id").as_int();
        for (pugi::xpath_node reg_elem_member : lanelet.node().select_nodes("member[@type='relation' and @role='regulatory_element']"))
        {
            int32_t ref = reg_elem_member.node().attribute("ref").as_int();
            lanelets_by_id[relation_id]->regulatory_elements().push_back(regulatory_elements_by_id[ref]);
        }
    }
}

void XMLParser::parse_reference_points()
{
    for (pugi::xpath_node relation : doc.select_nodes("//relation/tag[@v='reference' and @k='type']/.."))
    {
        // std::cout << "found reference" << std::endl;

        for (pugi::xpath_node member : relation.node().select_nodes((boost::format("member[@type='node' and @role='%s']") % "boundary").str().c_str()))
        {
            int32_t id = member.node().attribute("ref").as_int();
            // std::cout << "ref id: " << id << std::endl;
            drivable_boundary.push_back(points_by_id[id]);
        }

        // check for origin reference point
        for (pugi::xpath_node member : relation.node().select_nodes((boost::format("member[@type='node' and @role='%s']") % "origin").str().c_str()))
        {
            int32_t id = member.node().attribute("ref").as_int();
            local_origin_defined = true;
            reference_origin = points_by_id[id];
            // std::cout << "ref id: " << id << std::endl;
        }
    }
}

void XMLParser::parse_center_lines()
{
    for (pugi::xpath_node relation : doc.select_nodes("//relation/tag[@v='center_line' and @k='type']/.."))
    {
        TagWalker tagwalker;
        relation.node().traverse(tagwalker);
        AttributeMap attmap = tagwalker.attributes;
        // std::cout << "found center line: " << attmap["name"].as_string() << std::endl;

        reference_line_t center_line;
        for (pugi::xpath_node member : relation.node().select_nodes((boost::format("member[@type='node' and @role='%s']") % "point").str().c_str()))
        {
            int32_t id = member.node().attribute("ref").as_int();
            // std::cout << "ref id: " << id << std::endl;
            center_line.push_back(points_by_id[id]);
        }
        // std::cout << "point number: " << center_line.size() << std::endl;
        center_lines.insert(std::make_pair(attmap["name"].as_string(), center_line));
    }
}
} // namespace

std::vector<lanelet_ptr_t> LLet::parse_xml(std::string filename)
{
    pugi::xml_document doc;
    pugi::xml_parse_result res = doc.load_file(filename.c_str());

    XMLParser parser(doc);

    //    std::cout << "got " << parser.points_by_id.size() << " points." << std::endl;
    //    std::cout << "got " << parser.linestrips_by_id.size() << " line strips." << std::endl;
    //    std::cout << "got " << parser.lanelets.size() << " lanelets." << std::endl;

    return parser.parsed_lanelets();
}

LaneletXML LLet::parse_xml_full(std::string filename)
{
    pugi::xml_document doc;
    pugi::xml_parse_result res = doc.load_file(filename.c_str());

    XMLParser parser(doc);

    //    std::cout << "got " << parser.points_by_id.size() << " points." << std::endl;
    //    std::cout << "got " << parser.linestrips_by_id.size() << " line strips." << std::endl;
    //    std::cout << "got " << parser.lanelets.size() << " lanelets." << std::endl;
    LaneletXML xml_struct;
    xml_struct.lanelets = parser.parsed_lanelets();
    xml_struct.local_origin_defined = parser.parsed_reference_origin(&xml_struct.reference_origin);
    xml_struct.drivable_boundary = parser.parsed_boundary_points();
    xml_struct.center_lines = parser.parsed_center_lines();

    return xml_struct;
}