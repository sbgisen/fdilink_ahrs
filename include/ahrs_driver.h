#ifndef BASE_DRIVER_H_
#define BASE_DRIVER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <serial/serial.h> //ROS的串口包 http://wjwwood.io/serial/doc/1.1.0/index.html
#include <math.h>
#include <fstream>
#include <fdilink_data_struct.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Pose2D.h>
#include <boost/thread.hpp>
#include <string>
#include <ros/package.h>
#include <crc_table.h>
#include <dynamic_reconfigure/server.h>
#include <fdilink_ahrs/FdilinkAhrsConfig.h>


using namespace std;


namespace FDILink
{
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define IMU_LEN  0x38   //56
#define AHRS_LEN 0x30   //48
#define INSGPS_LEN 0x54 //84
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295
// Sample covariance
#define IMU_MAG_COV {0.01, 0.01, 0.01}
#define IMU_GYRO_COV {0.01, 0.01, 0.01}
#define IMU_ACCEL_COV {0.05, 0.05, 0.05}

class ahrsBringup
{
public:
  ahrsBringup();
  ~ahrsBringup();
  void processLoop();
  bool checkCS8(int len);
  bool checkCS16(int len);
  void checkSN(int type);
  void magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz);
  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<fdilink_ahrs::FdilinkAhrsConfig> reconfig_server_;
  void reconfigCallback(fdilink_ahrs::FdilinkAhrsConfig &config, uint32_t level);

private:
  bool if_debug_;
  //sum info
  int sn_lost_ = 0;
  int crc_error_ = 0;
  uint8_t read_sn_ = 0;
  bool frist_sn_;
  int device_type_ = 1;

  //covariance info
  std::vector<double> imu_mag_cov;
  std::vector<double> imu_gyro_cov;
  std::vector<double> imu_accel_cov;

  //serial
  serial::Serial serial_; //声明串口对象
  std::string serial_port_;
  int serial_baud_;
  int serial_timeout_;
  //data
  FDILink::imu_frame_read  imu_frame_;
  FDILink::ahrs_frame_read ahrs_frame_;
  FDILink::insgps_frame_read insgps_frame_;

  //frame name
  string imu_frame_id_;

  //topic
  string imu_topic_, mag_pose_2d_topic_, imu_topic_trueEast_, mag_topic_;

  //Publisher
  ros::Publisher imu_pub_;
  ros::Publisher imu_trueEast_pub_;
  ros::Publisher mag_pose_pub_;
  ros::Publisher mag_pub_;

  // others
  double yaw_offset;
  tf::Quaternion q_rot;
  double mag_offset_x_;
  double mag_offset_y_;
  double mag_offset_z_;
  double mag_covariance_;
}; //ahrsBringup
} // namespace FDILink

#endif
