#include "Road.h"
#include "Lanes.h"

#include <algorithm>
#include <cctype>
#include <math.h>
#include <stdio.h>

namespace odr
{

double Crossfall::get_crossfall(double s, double t) const
{
    std::shared_ptr<const Poly3> poly = this->get_poly(s);
    if (poly)
    {
        Side side = Side::Both;
        if (this->sides.find(poly->s0) != this->sides.end())
            side = this->sides.at(poly->s0);

        if (t > 0 /*left*/ && side == Side::Right)
            return 0;
        else if (t < 0 /*right*/ && side == Side::Left)
            return 0;

        return poly->get(s) * -static_cast<double>(sign(t));
    }

    return 0;
}

Road::Road(double length, int id, int junction)
    : id(id), junction(junction), length(length) {}

Vec3D Road::get_xyz(double s, double t, double h) const
{
    const Mat3D trans_mat = this->get_transformation_matrix(s);
    const Vec3D xyz = MatVecMultiplication(trans_mat, Vec3D{t, h, 1});
    return xyz;
}

Vec3D Road::get_surface_pt(double s, double t) const
{
    std::shared_ptr<const Lane> lane = this->get_lane(s, t);
    if (!lane)
        this->get_xyz(s, t, 0);

    const double superelevation = this->superelevation.get(s);
    const double crossfall = this->crossfall.get_crossfall(s, t);

    double       h_offset = 0;
    const double dt = lane->get_border(s) - t;
    if (lane->level)
        h_offset = std::tan(-(superelevation + crossfall)) * dt;
    else
        h_offset = std::tan(crossfall) * dt;

    auto lane_iter = lane->lane_section->lanes.find(lane->id);
    while (lane_iter->second->id != 0)
    {
        const double lane_width = lane_iter->second->lane_width.get(s - lane->lane_section->s0);
        if (lane_iter->second->level)
            h_offset += std::tan(-(superelevation + crossfall)) * lane_width;
        else
            h_offset += std::tan(crossfall) * lane_width;
        lane_iter = (lane_iter->second->id > 0) ? std::prev(lane_iter) : std::next(lane_iter);
    }

    return this->get_xyz(s, t, h_offset);
}

Mat3D Road::get_transformation_matrix(double s) const
{
    const Vec3D  s_vec = this->ref_line->get_grad(s);
    const double superelevation = this->superelevation.get(s);

    const Vec3D e_t = normalize(Vec3D{-s_vec[1], s_vec[0], std::tan(superelevation) / s_vec[0]});
    const Vec3D e_h = normalize(crossProduct(s_vec, e_t));
    const Vec3D p0 = this->ref_line->get_xyz(s);

    const Mat3D trans_mat{{{e_t[0], e_h[0], p0[0]},
                           {e_t[1], e_h[1], p0[1]},
                           {e_t[2], e_h[2], p0[2]}}};

    return trans_mat;
}

std::shared_ptr<const Lane> Road::get_lane(double s, double t) const
{
    std::shared_ptr<const LaneSection> lane_section = this->get_lane_section(s);
    if (!lane_section)
        return nullptr;
    return lane_section->get_lane(s, t);
}

std::shared_ptr<const LaneSection> Road::get_lane_section(double s) const
{
    std::shared_ptr<const LaneSection> lane_section = nullptr;
    if (this->lane_sections.size() > 0)
    {
        auto target_lane_sec_iter = this->lane_sections.upper_bound(s);
        if (target_lane_sec_iter != this->lane_sections.begin())
            target_lane_sec_iter--;
        lane_section = target_lane_sec_iter->second;
    }
    return lane_section;
}

} // namespace odr