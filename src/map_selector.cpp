#include "map_selector.hpp"
#include "utility.hpp"

#include <yaml-cpp/yaml.h>
#include <map_server/image_loader.h>
#include <nav_msgs/SetMap.h>

#include <fstream>
#include <libgen.h>

template <typename T>
void operator>>(const YAML::Node &node, T &i)
{
    i = node.as<T>();
}

MapSelector::MapSelector()
{
    std::string frame_id{getParamFromRosParam<std::string>(nhp_, "frame_id")};
    
    // read map information
    XmlRpc::XmlRpcValue map_list;
    nhp_.getParam("map_list", map_list);
    ROS_ASSERT(map_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_INFO("number of map info: %i", (int)map_list.size());

    // read map params
    // origin of map 0 is used as global origin
    LLA zero_map_lla;
    for (int32_t i = 0; i < map_list.size(); ++i)
    {
        MapInfo map_info;
        map_info.frame_id = "map" + std::to_string(i);

        auto read_string = [&](std::string key)
        {
            std::string ret;
            if (not map_list[i][key].valid())
                ROS_ERROR("%d th %s is not specified", int(i), key.c_str());
            if (map_list[i][key].getType() != XmlRpc::XmlRpcValue::TypeString)
                ROS_ERROR("%d th %s's type must be string", int(i), key.c_str());
            ret = static_cast<std::string>(map_list[i][key]);
            ROS_INFO("%s: %s", key.c_str(), ret.c_str());
            return ret;
        };
        map_info.path = read_string("path");
        map_info.obj_map_path = read_string("obj_map_path");

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
    // initialize ros objects
    sub_pose_ = nhp_.subscribe("/amcl_pose", 1, &MapSelector::subPoseCallback, this);

    pub_obj_map_ = nhp_.advertise<nav_msgs::OccupancyGrid>("obj_map", 1);
    pub_map_ = nhp_.advertise<nav_msgs::OccupancyGrid>("map", 1);
    for (int i = 0; i < maps_.size(); ++i)
        pub_maps_.push_back(nhp_.advertise<nav_msgs::OccupancyGrid>("map" + std::to_string(i), 1));

    ros::NodeHandle nh;
    auto trgps_srv = nhp_.advertiseService("transform_gps_pose", &MapSelector::transformGPSCallback, this);
    auto chmap_srv = nhp_.advertiseService("change_map", &MapSelector::changeMapCallback, this);
    cli_set_map_ = nh.serviceClient<nav_msgs::SetMap>("set_map");

    tf2_ros::TransformListener tfl {tf_buffer_};
    
    ros::Rate rate{5};
    while (ros::ok())
    {
        publishMaps();
        publishTransforms();

        ros::spinOnce();
        rate.sleep();
    }
}

bool MapSelector::transformGPSCallback(
    map_selector::transform_gps_pose::Request &req,
    map_selector::transform_gps_pose::Response &res
)
{
    try
    {
        geometry_msgs::PoseStamped in;
        geometry_msgs::PoseStamped out;
        in.header.frame_id = "gps";
        in.header.stamp = ros::Time(0);
        in.pose = req.gps_pose;
        geometry_msgs::TransformStamped trans = tf_buffer_.lookupTransform(
            "map" + std::to_string(current_map_),
            "gps",
            in.header.stamp,
            ros::Duration(0.5)
        );
        tf2::doTransform(in, out, trans);
        res.map_pose = out.pose;
    }
    catch (tf2::TransformException& e)
    {
        ROS_WARN("transform gps pose exception: %s", e.what());
    }
    return true;
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

    try
    {
        geometry_msgs::PoseStamped in;
        geometry_msgs::PoseStamped out;
        geometry_msgs::PoseWithCovarianceStamped out_msg;
        in.header = recent_pose_.header;
        in.header.stamp = ros::Time(0);
        in.pose = recent_pose_.pose.pose;
        geometry_msgs::TransformStamped trans = tf_buffer_.lookupTransform(
            "map" + std::to_string(num),
            recent_pose_.header.frame_id,
            in.header.stamp,
            ros::Duration(0.5)
        );
        tf2::doTransform(in, out, trans);
        out_msg = recent_pose_;
        out_msg.header = out.header;
        out_msg.pose.pose = out.pose;
        ROS_INFO("tr pose frame id: %s", out_msg.header.frame_id.c_str());
        ROS_INFO("tr pos x: %f, y: %f", out_msg.pose.pose.position.x, out_msg.pose.pose.position.y);

        out_msg.header.frame_id = "map";
        nav_msgs::SetMap set_map;
        set_map.request.initial_pose = out_msg;
        auto cpmap = maps_[num];
        cpmap.header.frame_id = "map";
        set_map.request.map = cpmap;
        if (cli_set_map_.call(set_map))
            ROS_INFO("Succeeded to call set_map");
        else
        {
            ROS_ERROR("failed to call set_map");
        }
    }
    catch (tf2::TransformException& e)
    {
        ROS_WARN("pose_transform: %s", e.what());
    }

    res.response = 1;
    return true;
}

void MapSelector::readMaps(std::string frame_id)
{
    for (int i = 0; i < map_infos_.size(); ++i)
    {
        auto cpmap = readMap(map_infos_[i].path);
        cpmap.header.frame_id = "map" + std::to_string(i);
        maps_.push_back(cpmap);

        obj_maps_.push_back(readMap(map_infos_[i].obj_map_path));
    }
    ROS_INFO("read %d maps", (int)maps_.size());
}

nav_msgs::OccupancyGrid MapSelector::readMap(std::string path)
{
    std::string files_str;
    std::string mapfname;
    double res;
    double origin[3], height;
    int negate;
    double occ_th, free_th;
    MapMode mode;

    ROS_INFO("Map %s", path.c_str());
    std::ifstream fin(path);
    if (fin.fail())
    {
        ROS_ERROR("Map_server could not open %s.", path.c_str());
        ros::shutdown();
        exit(0);
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
        exit(0);
    }
    try
    {
        doc["negate"] >> negate;
    }
    catch (YAML::InvalidScalar)
    {
        ROS_ERROR("The map does not contain a negate tag or it is invalid.");
        ros::shutdown();
        exit(0);
    }
    try
    {
        doc["occupied_thresh"] >> occ_th;
    }
    catch (YAML::InvalidScalar)
    {
        ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
        ros::shutdown();
        exit(0);
    }
    try
    {
        doc["free_thresh"] >> free_th;
    }
    catch (YAML::InvalidScalar)
    {
        ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
        ros::shutdown();
        exit(0);
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
        exit(0);
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
            exit(0);            
        }
        if (mapfname[0] != '/')
        {
            // dirname can modify what you pass it
            char *fname_copy = strdup(path.c_str());
            mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
            free(fname_copy);
        }
    }
    catch (YAML::InvalidScalar)
    {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        ros::shutdown();
        exit(0);
    }
    ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
    nav_msgs::GetMap::Response map_resp;
    map_server::loadMapFromFile(&map_resp,
                                mapfname.c_str(), res, negate, occ_th, free_th, origin, mode);
    map_resp.map.info.origin.position.z = height;
    map_resp.map.info.map_load_time = ros::Time::now();
    map_resp.map.header.stamp = ros::Time::now();
    map_resp.map.header.frame_id = "map";
    ROS_INFO("Read a %d X %d map @ %.3lf m/cell", map_resp.map.info.width, map_resp.map.info.height,map_resp.map.info.resolution);
    return map_resp.map;
}


void MapSelector::publishMaps()
{
    for (int i = 0; i < pub_maps_.size(); ++i)
        pub_maps_[i].publish(maps_[i]);

    auto cpmap = maps_[current_map_];
    cpmap.header.frame_id = "map";
    pub_map_.publish(cpmap);

    pub_obj_map_.publish(obj_maps_[current_map_]);
}

void MapSelector::publishTransforms()
{
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
        tf2::Transform tr(zero, tf2::Vector3(map_info.x, map_info.y, height * 0.01));
        tf2::Quaternion quat; quat.setRPY(0.0, 0.0, map_info.rad);
        tf2::Transform ro(quat);

        tf2::convert(tr * ro, trans.transform);
        //tf2::convert(ro * tr, trans.transform);
        tfb_.sendTransform(trans);

        if (i == current_map_)
        {
            auto cptrans = trans;
            cptrans.child_frame_id = "map";
            tfb_.sendTransform(cptrans);
        }
    }
}

void MapSelector::subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
    recent_pose_ = *pose;
}