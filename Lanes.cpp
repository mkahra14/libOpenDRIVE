#include "Lanes.h"

#include <cmath>

namespace odr
{

Lane::Lane(int id, std::string type, bool level)
    : id(id), type(type), level(level)
{
}

double Lane::get_border(double s) const
{
    double t = 0.0;
    auto   lane_iter = this->lane_section->lanes.find(this->id);
    while (lane_iter->second->id != 0)
    {
        t += lane_iter->second->lane_width.get(s - this->lane_section->s0);
        lane_iter = (lane_iter->second->id > 0) ? std::prev(lane_iter) : std::next(lane_iter);
    }
    t = (this->id < 0) ? -t : t;

    const double t_offset = this->lane_section->road->lane_offset.get(s);
    return t + t_offset;
}

LaneSection::LaneSection(double s0)
    : s0(s0)
{
}

std::shared_ptr<const Lane> LaneSection::get_lane(double s, double t) const
{
    std::map<int, double> lane_borders = this->get_lane_borders(s);
    if (lane_borders.empty())
        return nullptr;

    std::map<double, int> border_to_lane;
    for (const auto &entry : lane_borders)
        border_to_lane[entry.second] = entry.first;

    auto brdr_iter = border_to_lane.upper_bound(t);

    if (brdr_iter == border_to_lane.end())
    {
        brdr_iter--;
    }
    else if (brdr_iter != border_to_lane.begin())
    {
        if (brdr_iter->second <= 0)
            brdr_iter--;
        else if (std::prev(brdr_iter)->second != 0 && std::abs(std::prev(brdr_iter)->first - t) < 1e-9)
            brdr_iter--;
    }

    return this->lanes.at(brdr_iter->second);
}

std::map<int, double> LaneSection::get_lane_borders(double s) const
{
    std::map<int, double> outer_borders;
    for (const auto &lane : this->lanes)
        outer_borders[lane.first] = lane.second->get_border(s);

    return outer_borders;
}

} // namespace odr