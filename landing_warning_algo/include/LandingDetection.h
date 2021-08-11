//
// Created by liuk on 2021/8/9.
//

#ifndef PRE_TAKEOFF_WARNING_LANDINGDETECTION_H
#define PRE_TAKEOFF_WARNING_LANDINGDETECTION_H
#include "pointcloud_common.h"

class LandingDetection {
public:
    LandingDetection(){};
    std::shared_ptr<pointcloud_common::occupied_grid3d> grid;

};


#endif //PRE_TAKEOFF_WARNING_LANDINGDETECTION_H
