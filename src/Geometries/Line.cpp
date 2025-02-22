#include "Geometries/Line.h"
#include "Geometries/RoadGeometry.h"
#include "Math.hpp"
#include "Utils.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <vector>

namespace odr
{
Line::Line(double s0, double x0, double y0, double hdg0, double length) : RoadGeometry(s0, x0, y0, hdg0, length, GeometryType::Line) {}

Vec2D Line::get_xy(double s) const
{
    const double x = (std::cos(hdg0) * (s - s0)) + x0;
    const double y = (std::sin(hdg0) * (s - s0)) + y0;
    return Vec2D{x, y};
}

Vec2D Line::get_grad(double s) const { return {{std::cos(hdg0), std::sin(hdg0)}}; }

std::set<double> Line::approximate_linear(double eps) const {
    std::set<double> res;
    for (double s = s0; s <= s0 + length; s += eps)
        res.insert(s);
    return res;
}

} // namespace odr
