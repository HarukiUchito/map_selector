#include "map_selector.hpp"
#include "utility.hpp"

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <map_server/image_loader.h>

#include <fstream>
#include <libgen.h>

template <typename T>
void operator>>(const YAML::Node &node, T &i)
{
    i = node.as<T>();
}

MapSelector::MapSelector()
{
    ros::NodeHandle nhp{"~"};
    std::string frame_id{getParamFromRosParam<std::string>(nhp, "frame_id")};
    
    // read map information
    XmlRpc::XmlRpcValue map_list;
    nhp.getParam("map_list", map_list);
    ROS_ASSERT(map_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_INFO("number of map info: %i", (int)map_list.size());

    // read map params
    // origin of map 0 is used as global origin
    LLA zero_map_lla;
    for (int32_t i = 0; i < map_list.size(); ++i)
    {
        MapInfo map_info;
        map_info.frame_id = "map" + std::to_string(i);
        if (map_list[i]["path"].getType() == XmlRpc::XmlRpcValue::TypeString)
            map_info.path = static_cast<std::string>(map_list[i]["path"]);
        ROS_INFO("path: %s", map_info.path.c_str());

        auto read_double = [&](std::string key)
        {
            double ret;
            if (not map_list[i][key].valid())
                ROS_ERROR("%d th %s is not specified", int(i), key.c_str());
            if (map_list[i][key].getType() != XmlRpc::XmlRpcValue::TypeDouble)
                ROS_ERROR("%d th %s's type must be double", int(i), key.c_str());
            ret = static_cast<double>(map_list[i][key]);
            ROS_INFO("%s: %f", key.c_str(), ret);
            return ret;
        };

        LLA lla(read_double("lat"), read_double("lon"), read_double("alt"));
        if (i == 0)
            zero_map_lla = lla;
        auto pos = CalcRelativePosition(zero_map_lla, lla);
        map_info.x = pos.x, map_info.y = pos.y;
        ROS_INFO("pos x: %f, y: %f", pos.x, pos.y);

        map_info.rad = read_double("rad");

        map_infos_.push_back(map_info);
    }
    // read actual maps
    readMaps(frame_id);

    if (maps_.size() > 0)
        current_map_ = 0;
}

void MapSelector::run()
{
    ros::NodeHandle nhp{"~"};

    auto pub_map{nhp.advertise<nav_msgs::OccupancyGrid>("map", 1)};
    std::vector<ros::Publisher> pub_maps;
    for (int i = 0; i < maps_.size(); ++i)
        pub_maps.push_back(nhp.advertise<nav_msgs::OccupancyGrid>("map" + std::to_string(i), 1));

    auto _ = nhp.advertiseService("change_map", &MapSelector::changeMapCallback, this);

    tf2_ros::TransformBroadcaster tfb;

    ros::Rate rate{5};
    while (ros::ok())
    {
        for (int i = 0; i < pub_maps.size(); ++i)
            pub_maps[i].publish(maps_[i]);
        auto cpmap = maps_[current_map_];
        cpmap.header.frame_id = "map";
        pub_map.publish(cpmap);

        double cnt = 1.0;
        for (int i = 0; i < map_infos_.size(); ++i)
        {
            MapInfo map_info {map_infos_[i]};

            geometry_msgs::TransformStamped trans;
            trans.header.frame_id = "gps";
            trans.child_frame_id = map_info.frame_id;
            trans.header.stamp = ros::Time::now();

            int height = 0;
            if (i != current_map_)
                height = -(i+1);

            tf2::Quaternion zero; zero.setRPY(0.0, 0.0, 0.0);
            tf2::Transform tr(zero, tf2::Vector3(map_info.x, map_info.y, height));
            tf2::Quaternion quat; quat.setRPY(0.0, 0.0, map_info.rad);
            tf2::Transform ro(quat);

            tf2::convert(tr * ro, trans.transform);
            tfb.sendTransform(trans);

            if (i == current_map_)
            {
                auto cptrans = trans;
                cptrans.child_frame_id = "map";
                tfb.sendTransform(cptrans);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}

bool MapSelector::changeMapCallback(
    map_selector::change_map::Request &req,
    map_selector::change_map::Response &res
)
{
    int num = req.map_number;
    if (num < 0 or maps_.size() <= num)
    {
        ROS_ERROR("new map number is out of range");
        res.response = 0;
        return false;
    }

    current_map_ = num;

    res.response = 1;
    return true;
}

void MapSelector::readMaps(std::string frame_id)
{
    std::string files_str;
    std::string mapfname;
    double res;
    double origin[3], height;
    int negate;
    double occ_th, free_th;
    MapMode mode;

    for (int i = 0; i < map_infos_.size(); ++i)
    {
        std::string file {map_infos_[i].path};
        ROS_INFO("Map %d: %s", i, file.c_str());
        std::ifstream fin(file);
        if (fin.fail())
        {
            ROS_ERROR("Map_server could not open %s.", file.c_str());
            ros::shutdown();
            return;
        }
        YAML::Node doc = YAML::Load(fin);
        try
        {
            doc["resolution"] >> res;
        }
        catch (YAML::InvalidScalar)
        {
            ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
            ros::shutdown();
            return;
        }
        try
        {
            doc["negate"] >> negate;
        }
        catch (YAML::InvalidScalar)
        {
            ROS_ERROR("The map does not contain a negate tag or it is invalid.");
            ros::shutdown();
            return;
        }
        try
        {
            doc["occupied_thresh"] >> occ_th;
        }
        catch (YAML::InvalidScalar)
        {
            ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
            ros::shutdown();
            return;
        }
        try
        {
            doc["free_thresh"] >> free_th;
        }
        catch (YAML::InvalidScalar)
        {
            ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
            ros::shutdown();
            return;
        }
        try
        {
            std::string modeS = "";
            doc["mode"] >> modeS;

            if (modeS == "trinary")
                mode = TRINARY;
            else if (modeS == "scale")
                mode = SCALE;
            else if (modeS == "raw")
                mode = RAW;
            else
            {
                ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
                exit(-1);
            }
        }
        catch (YAML::Exception)
        {
            ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming trinary");
            mode = TRINARY;
        }
        try
        {
            doc["origin"][0] >> origin[0];
            doc["origin"][1] >> origin[1];
            doc["origin"][2] >> origin[2];
        }
        catch (YAML::InvalidScalar)
        {
            ROS_ERROR("The map does not contain an origin tag or it is invalid.");
            ros::shutdown();
            return;
        }
        try
        {
            doc["height"] >> height;
        }
        catch (YAML::Exception)
        {
            height = 0;
        }
        try
        {
            doc["image"] >> mapfname;
            // TODO(at-wat): make this path-handling more robust
            if (mapfname.size() == 0)
            {
                ROS_ERROR("The image tag cannot be an empty string.");
                ros::shutdown();
                return;
            }
            if (mapfname[0] != '/')
            {
                // dirname can modify what you pass it
                char *fname_copy = strdup(file.c_str());
                mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
                free(fname_copy);
            }
        }
        catch (YAML::InvalidScalar)
        {
            ROS_ERROR("The map does not contain an image tag or it is invalid.");
            ros::shutdown();
            return;
        }

        ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());

        nav_msgs::GetMap::Response map_resp;
        map_server::loadMapFromFile(&map_resp,
                                    mapfname.c_str(), res, negate, occ_th, free_th, origin, mode);
        map_resp.map.info.origin.position.z = height;
        map_resp.map.info.map_load_time = ros::Time::now();
        map_resp.map.header.frame_id = "map" + std::to_string(i);
        map_resp.map.header.stamp = ros::Time::now();
        ROS_INFO("Read a %d X %d map @ %.3lf m/cell", map_resp.map.info.width, map_resp.map.info.height, map_resp.map.info.resolution);
        maps_.push_back(map_resp.map);
    }
    ROS_INFO("read %d maps", (int)maps_.size());
}