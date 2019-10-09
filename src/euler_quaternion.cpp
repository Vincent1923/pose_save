#include <ros/ros.h>
#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <opencv2/stitching/detail/warpers.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "euler_quaternion");

  // 地图"/robot1/map"中机器人的位姿
  double position_x = -0.214872;  // 位置 (position_x, position_y, position_z)
  double position_y = 1.33488;
  double position_z = 0.0;
  double position_yaw = M_PI / 4;   // 朝向

  // 地图"/robot1/map"到地图"/robot2/map"的变换矩阵，包括平移向量和旋转向量（旋转向量用四元数表示）
  double translation_x = -0.33;  // 平移矩阵 (translation_x, translation_y, translation_z)
  double translation_y = 2.41;
  double translation_z = 0.0;
  double transform_yaw = M_PI;  // 旋转角度

  Eigen::Vector3d euler_angle_pose(position_yaw, 0.0, 0.0);
  Eigen::AngleAxisd roll_angle_pose(Eigen::AngleAxisd(euler_angle_pose(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitch_angle_pose(Eigen::AngleAxisd(euler_angle_pose(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yaw_angle_pose(Eigen::AngleAxisd(euler_angle_pose(0), Eigen::Vector3d::UnitZ()));

  Eigen::Vector3d euler_angle_tf(transform_yaw, 0.0, 0.0);
  Eigen::AngleAxisd roll_angle_tf(Eigen::AngleAxisd(euler_angle_tf(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitch_angle_tf(Eigen::AngleAxisd(euler_angle_tf(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yaw_angle_tf(Eigen::AngleAxisd(euler_angle_tf(0), Eigen::Vector3d::UnitZ()));

  // 欧拉角转四元数
  Eigen::Quaterniond quaternion_pose;
  Eigen::Quaterniond quaternion_tf;
  quaternion_pose = yaw_angle_pose * pitch_angle_pose * roll_angle_pose;
  quaternion_tf = yaw_angle_tf * pitch_angle_tf * roll_angle_tf;
  cout << "quaternion_pose = \n" << quaternion_pose.coeffs() << endl;  // 注意 coeffs的顺序是 (x, y, z, w)，w 为实部，前三者为虚部
  cout << "quaternion_tf = \n" << quaternion_tf.coeffs() << endl;

  // 四元数转欧拉角
  euler_angle_pose = quaternion_pose.matrix().eulerAngles(2, 1, 0);
  euler_angle_tf = quaternion_tf.matrix().eulerAngles(2, 1, 0);
  std::cout << "euler_angle_pose = \n" << euler_angle_pose / M_PI * 180 << std::endl;
  std::cout << "euler_angle_tf = \n" << euler_angle_tf / M_PI * 180 << std::endl;

  return 0;
}
