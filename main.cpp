#include "Lanes.h"
#include "OpenDriveMap.h"
#include "Road.h"
#include "Utils.hpp"

#include <iostream>
#include <memory>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "ERROR: too few arguments" << std::endl;
        return -1;
    }
    odr::OpenDriveMap odr(argv[1]);

    for (std::pair<const int, std::shared_ptr<odr::Road>> &road_w_id : odr.roads)
    {
        std::shared_ptr<odr::Road> road = road_w_id.second;
        for (auto lanesec_iter = road->lane_sections.begin(); lanesec_iter != road->lane_sections.end(); lanesec_iter++)
        {
            std::shared_ptr<odr::LaneSection> lanesec = lanesec_iter->second;

            double lanesec_len = (std::next(lanesec_iter) == road->lane_sections.end()) ? road->length - lanesec->s0 : std::next(lanesec_iter)->second->s0 - lanesec->s0;

            for (std::pair<const double, std::shared_ptr<odr::Lane>> lane : lanesec->lanes)
            {
                for (double s = lanesec->s0; s < (lanesec->s0 + lanesec_len); s += 0.5)
                    road->get_surface_pt(s, lane.second->get_border(s))[0];
            }
        }
    }

    std::cout << "Finished\n";

    return 0;
}
