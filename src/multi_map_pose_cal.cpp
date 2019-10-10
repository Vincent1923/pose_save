#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

using namespace std;

struct EndPose
{
  double translation_x;
  double translation_y;
  double yaw;
};

struct MapTransform
{
  int map_name;
  Eigen::Isometry3d transform;  // 虽然称为3d，实际上是 4x4 的矩阵
};

std::vector<MapTransform> transform_maps;
std::vector<struct EndPose> end_poses;

// 计算地图 map_1 到地图 map_N 的变换关系
void InitMapTransform()
{
  struct EndPose end_pose_tmp;

  end_pose_tmp.translation_x = -0.891;  // map_1 的结束点位姿
  end_pose_tmp.translation_y = 2.071;
  end_pose_tmp.yaw = 1.5773;
  end_poses.push_back(end_pose_tmp);

  end_pose_tmp.translation_x = 3.49;  // map_2 的结束点位姿
  end_pose_tmp.translation_y = 4.02;
  end_pose_tmp.yaw = 3.11759;
  end_poses.push_back(end_pose_tmp);

  MapTransform map_tf;

  // map_1 的变换矩阵
  map_tf.map_name = 1;
  map_tf.transform = Eigen::Isometry3d::Identity();
  transform_maps.push_back(map_tf);

  std::vector<EndPose>::iterator it = end_poses.begin();
  // map_2...map_N 相对于 map_1 的变换矩阵
  for (int i = 1; i < 3; i++)
  {
    // 结束点方向以欧拉角表示
    Eigen::Vector3d euler_angle(it->yaw, 0.0, 0.0);
    // 结束点方向以旋转向量表示
    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(euler_angle(2), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(euler_angle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(euler_angle(0), Eigen::Vector3d::UnitZ()));

    // 结束点方向以四元数来表示
    Eigen::Quaterniond quaternion;
    quaternion = yaw_angle * pitch_angle * roll_angle;
    cout << "quaternion = \n" << quaternion.coeffs() << endl << endl << endl;  // 注意 coeffs的顺序是 (x, y, z, w)，w 为实部，前三者为虚部

    Eigen::Isometry3d tf_end_pose = Eigen::Isometry3d::Identity();
    tf_end_pose.rotate(quaternion);  // 按照 quaternion_tf 进行旋转
    tf_end_pose.pretranslate(Eigen::Vector3d(it->translation_x, it->translation_y, 0.0));  // 平移向量
    cout << "Transform matrix = \n" << tf_end_pose.matrix() << endl << endl;

    map_tf.map_name = i + 1;
    map_tf.transform = transform_maps.back().transform.operator * (tf_end_pose);

    transform_maps.push_back(map_tf);

    it++;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_map_pose_cal");

  InitMapTransform();

  // 打印地图 map_1 到地图 map_N 的变换关系
  for (std::vector<struct MapTransform>::iterator it = transform_maps.begin();
       it != transform_maps.end(); it++)
  {
    ROS_WARN("map %d", it->map_name);
    Eigen::Vector3d translation = it->transform.translation();
    cout << "translation = \n" << translation << endl;
    Eigen::Quaterniond quaternion = Eigen::Quaterniond(it->transform.rotation());
    cout << "quaternion = \n" << quaternion.coeffs() << endl << endl;
  }

  // 根据 current_map 和 target_map 的 id，计算两张地图的变换矩阵
  int current_map = 1;
  int target_map = 2;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  std::vector<struct MapTransform>::iterator current_map_it = transform_maps.begin();
  std::vector<struct MapTransform>::iterator target_map_it = transform_maps.begin();

  for (std::vector<struct MapTransform>::iterator it = transform_maps.begin();
       it != transform_maps.end(); it++)
  {
    if (it->map_name == current_map)
    {
      current_map_it = it;
    }
    if (it->map_name == target_map)
    {
      target_map_it = it;
    }
  }

  if (current_map == 1)
  {
    // 从 map_1 切换到 map_N
    transform = target_map_it->transform.inverse();
    ROS_WARN("First case!");
  }
  else if (target_map == 1)
  {
    // 从 map_N 切换到 map_1
    transform = current_map_it->transform;
    ROS_WARN("Second case!");
  }
  else
  {
    transform = target_map_it->transform.inverse().operator * (
                    current_map_it->transform);
    ROS_WARN("Third case!");
  }

  // 打印变换矩阵 transform
  Eigen::Vector3d transform_translation = transform.translation();
  cout << "transform translation = \n" << transform_translation << endl;
  Eigen::Quaterniond transform_quaternion = Eigen::Quaterniond(transform.rotation());
  cout << "transform quaternion = \n" << transform_quaternion.coeffs() << endl << endl;

  // 地图"current_map"中机器人的位姿
  double position_x = -0.04;  // 位置 (position_x, position_y, position_z)
  double position_y = 2.43;
  double position_z = 0.0;
  double position_yaw = 3.0;   // 朝向

  // 机器人朝向以欧拉角表示
  Eigen::Vector3d euler_angle_pose(position_yaw, 0.0, 0.0);
  // 机器人朝向以旋转向量表示
  Eigen::AngleAxisd roll_angle_pose(Eigen::AngleAxisd(euler_angle_pose(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitch_angle_pose(Eigen::AngleAxisd(euler_angle_pose(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yaw_angle_pose(Eigen::AngleAxisd(euler_angle_pose(0), Eigen::Vector3d::UnitZ()));

  // 机器人朝向和旋转向量分别以四元数来表示
  Eigen::Quaterniond quaternion_pose;
  quaternion_pose = yaw_angle_pose * pitch_angle_pose * roll_angle_pose;
  cout << "pose quaternion = \n" << quaternion_pose.coeffs() << endl << endl << endl;  // 注意 coeffs的顺序是 (x, y, z, w)，w 为实部，前三者为虚部

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.rotate(quaternion_pose);
  pose.pretranslate(Eigen::Vector3d(position_x, position_y, position_z));
  cout << "pose matrix = \n" << pose.matrix() << endl << endl;

  // 机器人经过变换后的位姿
  Eigen::Isometry3d pose_after = transform.operator * (pose);
  cout << "pose_after matrix = \n" << pose_after.matrix() << endl << endl;

  // 机器人经过变换后的坐标和朝向
  Eigen::Vector3d pos_map2 = pose_after.translation();
  cout << "position = \n" << pos_map2 << endl;
  Eigen::Quaterniond quaternion_map2 = Eigen::Quaterniond(pose_after.rotation());
  cout << "quaternion = \n" << quaternion_map2.coeffs() << endl << endl;

  double x = pos_map2[0];
  double y = pos_map2[1];
  double yaw;

  Eigen::Vector3d euler_angle = quaternion_map2.matrix().eulerAngles(2, 1, 0);
  cout << "euler_angle = \n" << euler_angle << endl << endl;

  if (abs(abs(euler_angle[1]) - M_PI) < 0.001 && abs(abs(euler_angle[2]) - M_PI) < 0.001)
  {
    yaw = euler_angle[0] - M_PI;
  }
  else
  {
    yaw = euler_angle[0];
  }

  cout << "x = " << x << endl;
  cout << "y = " << y << endl;
  cout << "yaw = " << yaw << endl;

  return 0;
}
