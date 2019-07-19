//
// Created by Dimitriy Litvak on 2019-06-13.
//

#include "Road.h"

Road::Road(const string &mapFile) {
    std::ifstream in_map_(mapFile.c_str(), std::ifstream::in);

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map.map_waypoints_x.push_back(x);
        map.map_waypoints_y.push_back(y);
        map.map_waypoints_s.push_back(s);
        map.map_waypoints_dx.push_back(d_x);
        map.map_waypoints_dy.push_back(d_y);
    }
}

const Road::Map &Road::getMap() const {
    return this->map;
}
