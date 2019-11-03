#ifndef MAP_SELECTOR_HPP
#define MAP_SELECTOR_HPP

#include "lla.hpp"

#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <map_selector/change_map.h>

struct MapInfo {
    std::string frame_id;
    std::string path;
    double x;
    double y;
    double rad; // relative yaw to gnss coordinate
};

class MapSelector {
public:
    MapSelector();

    void run();
private:
    int current_map_ {-1};
    std::vector<nav_msgs::OccupancyGrid> maps_;
    void readMaps(std::string frame_id);

    bool changeMapCallback(
        map_selector::change_map::Request &req,
        map_selector::change_map::Response &res
    );

    std::vector<MapInfo> map_infos_;
};

#endif
