//
// Created by Dimitriy Litvak on 2019-06-13.
//

#ifndef PATH_PLANNING_ROAD_H
#define PATH_PLANNING_ROAD_H

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using std::string;
using std::vector;

class Road {
public:
    explicit Road(const string &mapFile);

    struct Map {
        vector<double> map_waypoints_x;
        vector<double> map_waypoints_y;
        vector<double> map_waypoints_s;
        vector<double> map_waypoints_dx;
        vector<double> map_waypoints_dy;
    };

    // The max s value before wrapping around the track back to 0
    constexpr static double MAX_S = 6945.554;

    constexpr static int NUM_LANES = 3;
    constexpr static double LANE_WIDTH = 4;
    constexpr static double SPEED_LIMIT_MPH = 49.5;
    constexpr static double SPEED_LIMIT_MPS = SPEED_LIMIT_MPH * 1609 / 3600;

    enum LANE : int {
        LEFT_LANE = 0, MIDDLE_LANE = 1, RIGHT_LANE = 2, INVALID_LANE = -1
    };

    const Map &getMap() const;
private:
    Map map;

};


#endif //PATH_PLANNING_ROAD_H
