#include <ros/ros.h>
#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "use_geometry");

  // 3D旋转矩阵直接使用 Matrix3d 或 Matrix3f
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
  // 旋转向量使用 AngleAxis
  Eigen::AngleAxisd rotation_vector (M_PI/4, Eigen::Vector3d(0, 0, 1));

  cout .precision(3);
  cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;

  rotation_matrix = rotation_vector.toRotationMatrix();

  // 用 AngleAxis 可以进行坐标变换
  Eigen::Vector3d v(1, 0, 0);
  Eigen::Vector3d v_rotated = rotation_vector * v;
  cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;
  // 或者用旋转矩阵
  v_rotated = rotation_matrix * v;
  cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;

  // 欧拉角：可以将旋转矩阵直接转换成欧拉角
  Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);  // ZYX顺序，即yaw pitch roll顺序
  cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

  // 欧式变换矩阵使用 Eigen:Isometry
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();  // 虽然称为3d，实际上是 4x4 的矩阵
  T.rotate(rotation_vector);  // 按照 rotation_vector 进行旋转
  T.pretranslate(Eigen::Vector3d(1, 3, 4));  // 把平移向量设成 (1, 3, 4)
  cout << "Transform matrix = \n" << T.matrix() << endl;

  // 用变换矩阵进行坐标变换
  Eigen::Vector3d v_transformed = T * v;  // 相当于 R*v+t
  cout << "v transformed = " << v_transformed.transpose() << endl;

  // 四元数
  // 可以直接把 AngleAxis 赋值给四元数，反之亦然
  Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
  cout << "quaternion = \n" << q.coeffs() << endl;  // 注意 coeffs的顺序是 (x, y, z, w)，w 为实部，前三者为虚部
  // 也可以把旋转矩阵赋值给它
  q = Eigen::Quaterniond(rotation_matrix);
  cout << "quaternion = \n" << q.coeffs() << endl;


  return 0;
}
