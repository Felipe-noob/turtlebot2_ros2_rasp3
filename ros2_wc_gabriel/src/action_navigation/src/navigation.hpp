#ifndef _NAVIGATION_H
#define _NAVIGATION_H

#include <vector>
#include <cstdint>

enum TurtlePosition_e {
    UNKNOWN = 0,
    INPUT_SIDE1 = 1,  // 1
    INPUT_SIDE2 = 2 ,  // 2
    OUTPUT_SIDE1 = 3 , // 3
    OUTPUT_SIDE2 = 4 // 4
  };

struct Point {
    double x;
    double y;
};

struct GlobalMap {
    const Point WP1 {0.0, 0.4};
    const Point WP2 {0.0, -0.4};
    const Point WP3 {3.6, 0.4};
    const Point WP4 {3.6, -0.4};

    const Point WP5 {0.9, 0.0};
    const Point WP6 {1.8, 0.0};
    const Point WP7 {2.7, 0.0};

    const Point WP1_1 {0.9, 0.6};
    const Point WP2_1 {0.9, -0.6};
    const Point WP3_1 {2.7, 0.6};
    const Point WP4_1 {2.7, -0.6};
};


std::vector<std::vector<double>> get_waypoints(uint32_t startID, uint32_t endID);

#endif