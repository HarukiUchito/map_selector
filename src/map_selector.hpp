#ifndef MAP_SELECTOR_HPP
#define MAP_SELECTOR_HPP

#include "lla.hpp"

#include <vector>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_selector/change_map.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

struct MapInfo {
    std::string frame_id;
    std::string path;
    std::string obj_map_path;
    double x;
    double y;
    double rad; // relative yaw to gnss coordinate
};

class MapSelector {
public:
    MapSelector();

    void run();
private:
    ros::NodeHandle nhp_ {"~"};

    ros::Subscriber sub_pose_;
    geometry_msgs::PoseWithCovarianceStamped recent_pose_;
    void subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);

    ros::Publisher pub_map_, pub_obj_map_;
    std::vector<ros::Publisher> pub_maps_;

    tf2_ros::TransformBroadcaster tfb_;
    tf2_ros::Buffer tf_buffer_;

    int current_map_ {-1};
    std::vector<nav_msgs::OccupancyGrid> maps_, obj_maps_;
    nav_msgs::OccupancyGrid readMap(std::string path);
    void readMaps(std::string frame_id);
    void publishMaps();

    ros::ServiceClient cli_set_map_;

    bool changeMapCallback(
        map_selector::change_map::Request &req,
        map_selector::change_map::Response &res
    );

    std::vector<MapInfo> map_infos_;
    void publishTransforms();
};

#endif
