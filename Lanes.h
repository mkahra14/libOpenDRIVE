#pragma once

#include "Road.h"

#include <map>
#include <memory>

namespace odr
{

struct Lane;

struct LaneSection : public std::enable_shared_from_this<LaneSection>
{
    LaneSection(double s0);
    std::shared_ptr<const Lane> get_lane(double s, double t) const;
    std::map<int, double>       get_lane_borders(double s) const;

    double                s0;
    std::shared_ptr<Road> road;

    std::map<int, std::shared_ptr<Lane>> lanes;
};

struct Lane : public std::enable_shared_from_this<Lane>
{
    Lane(int id, std::string type, bool level);
    double get_border(double s) const;

    int         id;
    std::string type;
    bool        level = false;

    int predecessor = 0;
    int successor = 0;

    std::shared_ptr<LaneSection> lane_section;
    CubicSpline                  lane_width;
};

} // namespace odr