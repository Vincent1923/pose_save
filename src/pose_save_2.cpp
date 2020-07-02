#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <string>
#include <fstream>

#include "pose_save/SavePose.h"

using std::string;

geometry_msgs::Pose pose;  // 当前机器人的位姿

bool savePoseCallback(pose_save::SavePose::Request& req, pose_save::SavePose::Response& res)
{
  std::string yamlFile;  // fileName = "/home/aicrobo/Documents/yaml_files/map_transform.yaml";

  yamlFile = req.fileName;

  ROS_INFO("fileName: %s", yamlFile.c_str());

  YAML::Node yaml;

  yaml["position_x"] = pose.position.x;
  yaml["position_y"] = pose.position.y;
  yaml["position_z"] = pose.position.z;
  yaml["orientation_x"] = pose.orientation.x;
  yaml["orientation_y"] = pose.orientation.y;
  yaml["orientation_z"] = pose.orientation.z;
  yaml["orientation_w"] = pose.orientation.w;

  try
  {
    std::ofstream fout(yamlFile.c_str());
    fout << yaml;
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR("Faild to save file: %s \n%s", yamlFile.c_str(), e.what());
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_save_node");
  ros::NodeHandle node;

  std::string base_frame = "base_footprint";
  std::string map_frame = "map";

  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  ros::ServiceServer savePoseService;
  savePoseService = node.advertiseService("save_pose", savePoseCallback);

  tf::TransformListener tf_listener;
  tf::Stamped <tf::Pose> ident_base;
  tf::Stamped <tf::Transform> map_to_base;

  double base_frame_subscribe_period = 0.5;
  ros::Rate r(1.0 / base_frame_subscribe_period);
  while (ros::ok())
  {
    ident_base.setIdentity();
    ident_base.frame_id_ = base_frame;
    ident_base.stamp_ = ros::Time(0);
    try
    {
      // ros::Time(0)指定了时间为0，即获得最新有效的变换
      tf_listener.waitForTransform(map_frame, base_frame, ros::Time(0), ros::Duration(5));
      tf_listener.transformPose(map_frame, ident_base, map_to_base);
    }
    catch (tf::TransformException e)
    {
      ROS_ERROR("Failed to get transform, exiting (%s)", e.what());
      exit(-1);
    }

    pose.position.x = map_to_base.getOrigin().x();
    pose.position.y = map_to_base.getOrigin().y();
    pose.position.z = map_to_base.getOrigin().z();
    pose.orientation.x = map_to_base.getRotation().x();
    pose.orientation.y = map_to_base.getRotation().y();
    pose.orientation.z = map_to_base.getRotation().z();
    pose.orientation.w = map_to_base.getRotation().w();

    std::cout << "getOrigin().x: " << map_to_base.getOrigin().x() << std::endl;
    std::cout << "getOrigin().y: " << map_to_base.getOrigin().y() << std::endl;
    std::cout << "getOrigin().z(): " << map_to_base.getOrigin().z() << std::endl;
    std::cout << "getRotation().x(): " << map_to_base.getRotation().x() << std::endl;
    std::cout << "getRotation().y(): " << map_to_base.getRotation().y() << std::endl;
    std::cout << "getRotation().z(): " << map_to_base.getRotation().z() << std::endl;
    std::cout << "getRotation().w(): " << map_to_base.getRotation().w() << std::endl;

    r.sleep();
  }

  ros::spin();

  return 0;
}
