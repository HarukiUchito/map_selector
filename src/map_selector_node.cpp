#include <ros/ros.h>
#include "map_selector.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_selector_node");
    
    MapSelector noen;
    noen.run();
    
    return 0;
}
