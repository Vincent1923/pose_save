/*
 * 机器人位姿坐标系变换计算：根据给定的两张地图坐标系的平移向量和四元数，计算机器人经过坐标系变换后的位姿。
 * autor: Yuming Liang
 * date: 2018-10-25
 */

#include <ros/ros.h>
#include <opencv2/stitching/detail/warpers.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

// Use Bullet's Quaternion object to create one from Euler angles
#include <LinearMath/btQuaternion.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "invert_pose_node");

/*
 * 已知机器人在地图"/robot1/map"的位姿pose，计算经过地图切换后在地图"/robot2/map"的位姿pose_warped
 */

  // 地图"/robot1/map"中机器人的位姿
  double position_x = -6.4324;
  double position_y = 4.3184;
  double position_z = 0.0;
  double orientation_x = 0.0;
  double orientation_y = 0.0;
  double orientation_z = 0.997495;
  double orientation_w = 0.0707372;

  // 地图"/robot1/map"到地图"/robot2/map"的平移向量和旋转向量（旋转向量用四元数表示）
  double translation_x = -4.93361;  // 平移矩阵(translation_x, translation_y, translation_z)
  double translation_y = 5.53478;
  double translation_z = 0.0;
  double quaternion_x = 0.0;     // 四元数(quaternion_x, quaternion_y, quaternion_z, quaternion_w)
  double quaternion_y = 0.0;
  double quaternion_z = -0.713266;
  double quaternion_w = 0.700893;

  tf::Transform transform = tf::Transform(tf::Quaternion(quaternion_x, quaternion_y, quaternion_z, quaternion_w),
                                          tf::Vector3(translation_x, translation_y, translation_z));

/*
 * 计算地图坐标系间的变换矩阵
 * 注意：由于要计算的位姿是相对于地图"/robot2/map"，而已知的变换是"/robot1/map"到"/robot2/map"的变换，
 *      因此首先需要计算它的逆变换，即"/robot2/map"到"/robot1/map"的变换。
 */
  tf::Transform inverse_transform = transform.inverse();

  // 获取"/robot2/map"到"/robot1/map"的平移向量和旋转矩阵（四元数）
  double tx = inverse_transform.getOrigin().getX();
  double ty = inverse_transform.getOrigin().getY();
  double x = inverse_transform.getRotation().x();
  double y = inverse_transform.getRotation().y();
  double z = inverse_transform.getRotation().z();
  double w = inverse_transform.getRotation().w();
  double s = 2.0 / (x * x + y * y + z * z + w * w);  // 四元数(x,y,z,w)转化为旋转矩阵
  double a = 1 - y * y * s - z * z * s;
  double b = x * y * s + z * w * s;

  // 利用平移向量和旋转矩阵创建"/robot2/map"到"/robot1/map"的变换矩阵
  cv::Mat transform_matrix = cv::Mat::eye(2, 3, CV_64F);  // 仿射变换矩阵，大小为2*3
  transform_matrix.at<double>(0, 0) = transform_matrix.at<double>(1, 1) = a;
  transform_matrix.at<double>(1, 0) = b;
  transform_matrix.at<double>(0, 1) = -b;
  transform_matrix.at<double>(0, 2) = tx;
  transform_matrix.at<double>(1, 2) = ty;

/*
 * 计算机器人经过变换后在"/robot2/map"中的朝向角度
 */
  // 计算地图坐标系"/robot2/map"到"/robot1/map"的旋转角度
  double roll_transform;
  double pitch_transform;
  double yaw_transform;  // 绕z轴的旋转角度
  tf::Quaternion quaternion(x, y, z, w);
  tf::Matrix3x3 matrix(quaternion);
  matrix.getRPY(roll_transform, pitch_transform, yaw_transform);

  // 计算机器人在"/robot1/map"中的朝向角度
  double roll, pitch, yaw;  // 机器人在"/robot1/map"中的朝向角度
  tf::Quaternion quaternion_pose(orientation_x, orientation_y,
                                 orientation_z, orientation_w);
  tf::Matrix3x3 matrix_pose(quaternion_pose);
  matrix_pose.getRPY(roll, pitch, yaw);

  // 计算机器人经过坐标变换后在"/robot2/map"中的朝向角度，并创建对应的四元数
  double yaw_warped;                 // 机器人在"/robot2/map"中的朝向角度
  yaw_warped = yaw + yaw_transform;
  tf::Quaternion quaternion_warped;  // 表示机器人在"/robot2/map"中朝向的四元数
//  quaternion_warped.setEulerZYX(yaw_warped, pitch, roll);  // 编译时会提示deprecated的warning，即函数setEulerZYX()已被弃用
  quaternion_warped.setRPY(roll, pitch, yaw_warped);

/*
 * 机器人进行地图切换后，计算它在地图"/robot2/map"中的位姿
 */
  geometry_msgs::PoseWithCovarianceStamped pose_warped;  // 机器人在"/robot2/map"中的位姿
  pose_warped.header.frame_id = "map";
//  pose_warped.header.stamp = ros::Time::now();
  pose_warped.pose.pose.position.x = position_x * transform_matrix.at<double>(0, 0)
                                     + position_y * transform_matrix.at<double>(0, 1)
                                     + transform_matrix.at<double>(0, 2);
  pose_warped.pose.pose.position.y = position_x * transform_matrix.at<double>(1, 0)
                                     + position_y * transform_matrix.at<double>(1, 1)
                                     + transform_matrix.at<double>(1, 2);
  pose_warped.pose.pose.position.z = 0.0;
  pose_warped.pose.pose.orientation.x = quaternion_warped.x();
  pose_warped.pose.pose.orientation.y = quaternion_warped.y();
  pose_warped.pose.pose.orientation.z = quaternion_warped.z();
  pose_warped.pose.pose.orientation.w = quaternion_warped.w();
  pose_warped.pose.covariance[6 * 0 + 0] = 0.5 * 0.5;
  pose_warped.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
  pose_warped.pose.covariance[6 * 5 + 5] = M_PI / 12.0 * M_PI / 12.0;

  ROS_ERROR("Publishing the pose_warped!");
  std::cout << "pose_warped.pose.pose.position.x: " << pose_warped.pose.pose.position.x << std::endl;
  std::cout << "pose_warped.pose.pose.position.y: " << pose_warped.pose.pose.position.y << std::endl;
  std::cout << "pose_warped.pose.pose.position.z: " << pose_warped.pose.pose.position.z << std::endl;
  std::cout << "pose_warped.pose.pose.orientation.x: " << pose_warped.pose.pose.orientation.x << std::endl;
  std::cout << "pose_warped.pose.pose.orientation.y: " << pose_warped.pose.pose.orientation.y << std::endl;
  std::cout << "pose_warped.pose.pose.orientation.z: " << pose_warped.pose.pose.orientation.z << std::endl;
  std::cout << "pose_warped.pose.pose.orientation.w: " << pose_warped.pose.pose.orientation.w << std::endl;

/*
 * setEuler(yaw, pitch, roll)和setEulerZYX(yaw, pitch, roll)两个函数的旋转弧度参数表示有所不同。
 * setEuler(yaw, pitch, roll)中的yaw表示的是绕Y轴旋转的弧度；而setEulerZYX(yaw, pitch, roll)中的yaw表示的是绕Z轴旋转的弧度。
 * 在编译时候，若使用函数setEulerZYX()，会提示deprecated的warning，即函数setEulerZYX()已被弃用。
 * (1)void tf::Quaternion::setEuler(const tfScalar & yaw, const tfScalar & pitch, const tfScalar & roll)
 *    Set the quaternion using Euler angles.
 *    Parameters:
 *         yaw	  Angle around Y
 *         pitch	Angle around X
 *         roll	  Angle around Z
 * (2)void tf::Quaternion::setEulerZYX(const tfScalar & yaw, const tfScalar & pitch, const tfScalar & roll)
 *    Set the quaternion using euler angles.
 *    Parameters:
 *         yaw	  Angle around Z
 *         pitch	Angle around Y
 *         roll	  Angle around X
 */

  tf::Quaternion q_wormhole(orientation_x, orientation_y, orientation_z, orientation_w);
  tf::Vector3 v_wormhole(position_x, position_y, position_z);
  tf::Transform wormhole_before_transform = tf::Transform(q_wormhole, v_wormhole);

  tf::Quaternion q(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
  tf::Vector3 vector( translation_x, translation_y, translation_z);
  tf::Transform T = tf::Transform(q, vector);
  tf::Transform T_inverse = T.inverse();

  tf::Matrix3x3 rotation_matrix = T.getBasis();
  std::cout << "rotation matrix =" << std::endl;
  for (int i = 0; i < 3; i++)
  {
    std::cout << rotation_matrix.getRow(i).getX() << " ";
    std::cout << rotation_matrix.getRow(i).getY() << " ";
    std::cout << rotation_matrix.getRow(i).getZ() << std::endl;
  }
  tf::Quaternion Q;
  rotation_matrix.getRotation(Q);
  std::cout << "quaternion =" << std::endl;
  std::cout << Q.x() << " " << Q.y() << " " << Q.z() << " " << Q.w() << std::endl;

  tf::Transform wormhole = T_inverse.operator*(wormhole_before_transform);

  ROS_ERROR("Publishing the wormhole_after_transform!");
  std::cout << "wormhole.getOrigin().x(): " << wormhole.getOrigin().x() << std::endl;
  std::cout << "wormhole.getOrigin().y(): " << wormhole.getOrigin().y() << std::endl;
  std::cout << "wormhole.getOrigin().z(): " << wormhole.getOrigin().z() << std::endl;
  std::cout << "wormhole.getRotation().x(): " << wormhole.getRotation().x() << std::endl;
  std::cout << "wormhole.getRotation().y(): " << wormhole.getRotation().y() << std::endl;
  std::cout << "wormhole.getRotation().z(): " << wormhole.getRotation().z() << std::endl;
  std::cout << "wormhole.getRotation().w(): " << wormhole.getRotation().w() << std::endl;


  tf::Quaternion quaternion_test(wormhole.getRotation().x(),
                                 wormhole.getRotation().y(),
                                 wormhole.getRotation().z(),
                                 wormhole.getRotation().w());
  tf::Matrix3x3 matrix_test(quaternion_test);
  double roll_transform_test;
  double pitch_transform_test;
  double yaw_transform_test;
  matrix_test.getEulerYPR(yaw_transform_test, pitch_transform_test, roll_transform_test);
  std::cout << "yaw_transform_test: " << yaw_transform_test << std::endl;
  std::cout << "pitch_transform_test: " << pitch_transform_test << std::endl;
  std::cout << "roll_transform_test: " << roll_transform_test << std::endl;

  tf::Matrix3x3 rotation_matrix_test;
  rotation_matrix_test.setEulerYPR( 3.06755 * -1.0, 0., 0. );
  tf::Quaternion quat;
  rotation_matrix_test.getRotation( quat );
  std::cout << "quat.getX(): " << quat.getX() << std::endl;
  std::cout << "quat.getY(): " << quat.getY() << std::endl;
  std::cout << "quat.getZ(): " << quat.getZ() << std::endl;
  std::cout << "quat.getW(): " << quat.getW() << std::endl;

  return 0;
}
