#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>

#include "pose_save/SavePose.h"

#include <fstream>

using std::string;

geometry_msgs::Pose pose;  // 当前机器人的位姿

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  pose.position.x = msg->pose.pose.position.x;
  pose.position.y = msg->pose.pose.position.y;
  pose.position.z = msg->pose.pose.position.z;
  pose.orientation.x = msg->pose.pose.orientation.x;
  pose.orientation.y = msg->pose.pose.orientation.y;
  pose.orientation.z = msg->pose.pose.orientation.z;
  pose.orientation.w = msg->pose.pose.orientation.w;
}

bool savePoseCallback(pose_save::SavePose::Request& req, pose_save::SavePose::Response& res)
{
  std::string yamlPath, fileBaseName;
  std::string fileName;

  yamlPath = req.pathName;
  fileBaseName = "map_transform";
  fileName = yamlPath + "/" + fileBaseName + ".yaml";

  ROS_INFO("yamlPath: %s", yamlPath.c_str());
  ROS_INFO("fileName: %s", fileName.c_str());

//  fileName = "/home/aicrobo/Documents/yaml_files/map_transform.yaml";

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
    std::ofstream fout(fileName.c_str());
    fout << yaml;
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR("Faild to save file: %s \n%s", fileName.c_str(), e.what());
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_save_node");
  ros::NodeHandle node;

  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 3.0;
  pose.orientation.x = 1.0;
  pose.orientation.y = 2.0;
  pose.orientation.z = 3.0;
  pose.orientation.w = 4.0;

  ros::Subscriber pose_sub;  // 接收机器人的实时位姿
  pose_sub = node.subscribe("odom", 10, poseCallback);

  ros::ServiceServer savePoseService;
  savePoseService = node.advertiseService("save_pose", savePoseCallback);

  ros::spin();

  return 0;
}
