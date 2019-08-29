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
  ros::init(argc, argv, "pose_cal");

  // 地图"/robot1/map"中机器人的位姿
  double position_x = -0.214872;  // 位置 (position_x, position_y, position_z)
  double position_y = 1.33488;
  double position_z = 0.0;
  double orientation_x = 0.0;     // 朝向 (orientation_x, orientation_y, orientation_z, orientation_w)
  double orientation_y = 0.0;
  double orientation_z = 0.894145;
  double orientation_w = 0.447777;

  // 地图"/robot1/map"到地图"/robot2/map"的变换矩阵，包括平移向量和旋转向量（旋转向量用四元数表示）
  double translation_x = -0.33;  // 平移矩阵 (translation_x, translation_y, translation_z)
  double translation_y = 2.41;
  double translation_z = 0.0;
  double quaternion_x = 0.0;     // 四元数 (quaternion_x, quaternion_y, quaternion_z, quaternion_w)
  double quaternion_y = 0.0;
  double quaternion_z = 0.71;
  double quaternion_w = 0.70;


  // 初始化四元数，四元数 Eigen::Quaterniond 的正确初始化顺序为 Eigen::Quaterniond(w, x, y, z)
  Eigen::Quaterniond quaternion_pose(orientation_w, orientation_x, orientation_y, orientation_z);
  Eigen::Quaterniond quaternion_tf(quaternion_w, quaternion_x, quaternion_y, quaternion_z);
  cout << "quaternion_pos = \n" << quaternion_pose.coeffs() << endl;  // 注意 coeffs的顺序是 (x, y, z, w)，w 为实部，前三者为虚部
  cout << "quaternion_tf = \n" << quaternion_tf.coeffs() << endl;

  // 初始化旋转矩阵
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = quaternion_tf.matrix();
  cout .precision(3);
  cout << "rotation matrix =\n" << rotation_matrix.matrix() << endl;

  // 欧式变换矩阵使用 Eigen:Isometry
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();  // 虽然称为3d，实际上是 4x4 的矩阵
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  T.rotate(quaternion_tf);  // 按照 quaternion_tf 进行旋转
  T.pretranslate(Eigen::Vector3d(translation_x, translation_y, translation_z));  // 平移向量
  cout << "Transform matrix = \n" << T.matrix() << endl;

  Eigen::Isometry3d T_inverse = T.inverse();  // 欧式变换矩阵的逆变换
  cout << "T_inverse matrix = \n" << T_inverse.matrix() << endl;

  pose.rotate(quaternion_pose);
  pose.pretranslate(Eigen::Vector3d(position_x, position_y, position_z));
  cout << "pose matrix = \n" << pose.matrix() << endl;

  Eigen::Isometry3d pose_after = T_inverse.operator * (pose);
  cout << "pose_after matrix = \n" << pose_after.matrix() << endl;

  Eigen::Vector3d pos_map2 = pose_after.translation();
  cout << "position = \n" << pos_map2 << endl;
  Eigen::Quaterniond quaternion_map2 = Eigen::Quaterniond(pose_after.rotation());
  cout << "quaternion = \n" << quaternion_map2.coeffs() << endl;



  return 0;
}
