#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_cal");

  // 地图"/robot1/map"中机器人的位姿
  double position_x = -6.4324;  // 位置 (position_x, position_y, position_z)
  double position_y = 4.3184;
  double position_z = 0.0;
  double position_yaw = 3.0;   // 朝向

  // 地图"/robot1/map"到地图"/robot2/map"的变换矩阵，包括平移向量和旋转向量（旋转向量用四元数表示）
  double translation_x = -6.41;  // 平移矩阵 (translation_x, translation_y, translation_z)
  double translation_y = 4.38;
  double translation_z = 0.0;
  double transform_yaw = 0;  // 旋转角度

  // 机器人朝向以欧拉角表示
  Eigen::Vector3d euler_angle_pose(position_yaw, 0.0, 0.0);
  // 机器人朝向以旋转向量表示
  Eigen::AngleAxisd roll_angle_pose(Eigen::AngleAxisd(euler_angle_pose(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitch_angle_pose(Eigen::AngleAxisd(euler_angle_pose(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yaw_angle_pose(Eigen::AngleAxisd(euler_angle_pose(0), Eigen::Vector3d::UnitZ()));

  // 旋转变换以欧拉角表示
  Eigen::Vector3d euler_angle_tf(transform_yaw, 0.0, 0.0);
  // 旋转变换以旋转向量表示
  Eigen::AngleAxisd roll_angle_tf(Eigen::AngleAxisd(euler_angle_tf(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitch_angle_tf(Eigen::AngleAxisd(euler_angle_tf(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yaw_angle_tf(Eigen::AngleAxisd(euler_angle_tf(0), Eigen::Vector3d::UnitZ()));

  // 机器人朝向和旋转向量分别以四元数来表示
  Eigen::Quaterniond quaternion_pose;
  Eigen::Quaterniond quaternion_tf;
  quaternion_pose = yaw_angle_pose * pitch_angle_pose * roll_angle_pose;
  quaternion_tf = yaw_angle_tf * pitch_angle_tf * roll_angle_tf;
//  quaternion_pose = Eigen::Quaterniond(0.037, 0, 0, -0.999);
//  quaternion_tf = Eigen::Quaterniond(1, 0, 0, 0);
  cout << "quaternion_pos = \n" << quaternion_pose.coeffs() << endl << endl << endl;  // 注意 coeffs的顺序是 (x, y, z, w)，w 为实部，前三者为虚部
  cout << "quaternion_tf = \n" << quaternion_tf.coeffs() << endl << endl << endl;

//  Eigen::Vector3d euler_angle_pose_test = quaternion_pose.matrix().eulerAngles(2, 1, 0);
//  cout << "euler_angle_pose_test = \n" << euler_angle_pose_test << endl << endl;

  // 欧式变换矩阵使用 Eigen:Isometry
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();  // 虽然称为3d，实际上是 4x4 的矩阵
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  T.rotate(quaternion_tf);  // 按照 quaternion_tf 进行旋转
  T.pretranslate(Eigen::Vector3d(translation_x, translation_y, translation_z));  // 平移向量
  cout << "Transform matrix = \n" << T.matrix() << endl << endl;

  Eigen::Isometry3d T_inverse = T.inverse();  // 欧式变换矩阵的逆变换
//  Eigen::Isometry3d T_inverse = T;
  cout << "T_inverse matrix = \n" << T_inverse.matrix() << endl << endl;

  pose.rotate(quaternion_pose);
  pose.pretranslate(Eigen::Vector3d(position_x, position_y, position_z));
  cout << "pose matrix = \n" << pose.matrix() << endl << endl;

  // 机器人经过变换后的位姿
  Eigen::Isometry3d pose_after = T_inverse.operator * (pose);
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
