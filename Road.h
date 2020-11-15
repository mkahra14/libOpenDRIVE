#pragma once

#include "Geometries/Geometries.h"
#include "RefLine.h"
#include "Utils.hpp"

#include <map>
#include <memory>
#include <vector>

namespace odr
{

struct Lane;
struct LaneSection;
struct LaneOffset;

struct Crossfall : public CubicSpline
{
    enum Side
    {
        Both,
        Left,
        Right
    };

    Crossfall() = default;
    double get_crossfall(double s, double t) const;

    std::map<double, Side> sides;
};

struct RoadLink
{
    int elementId = -1;

    std::string elementType;
    std::string contactPoint;
};

struct RoadNeighbor
{
    int elementId = -1;

    std::string side;
    std::string direction;
};

struct SpeedRecord
{
    double      max = -1;
    std::string unit;
};

class Road : public std::enable_shared_from_this<Road>
{
public:
    Road(double length, int id, int junction);

    Vec3D get_xyz(double s, double t, double h) const;
    Vec3D get_surface_pt(double s, double t) const;
    Mat3D get_transformation_matrix(double s) const;

    std::shared_ptr<const Lane>        get_lane(double s, double t) const;
    std::shared_ptr<const LaneSection> get_lane_section(double s) const;

    int    id, junction;
    double length;

    RoadLink                  predecessor;
    RoadLink                  successor;
    std::vector<RoadNeighbor> neighbors;

    std::map<double, std::string> type;
    std::map<double, SpeedRecord> speed;

    std::shared_ptr<RefLine> ref_line;

    std::map<double, std::shared_ptr<LaneSection>> lane_sections;

    CubicSpline lane_offset;
    CubicSpline superelevation;
    Crossfall   crossfall;
};

} // namespace odr