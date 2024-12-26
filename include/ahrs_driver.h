#ifndef BASE_DRIVER_H_
#define BASE_DRIVER_H_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <iostream>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/detail/magnetic_field__struct.hpp>
#include <serial_driver/serial_driver.hpp>
#include <math.h>
#include <fstream>
#include <fdilink_data_struct.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <boost/thread.hpp>
#include <string>
#include <crc_table.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

namespace FDILink
{
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define IMU_LEN 0x38     // 56
#define AHRS_LEN 0x30    // 48
#define INSGPS_LEN 0x54  // 84
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295
// Sample covariance
#define IMU_MAG_COV                                                                                                    \
  {                                                                                                                    \
    0.01, 0.01, 0.01                                                                                                   \
  }
#define IMU_GYRO_COV                                                                                                   \
  {                                                                                                                    \
    0.01, 0.01, 0.01                                                                                                   \
  }
#define IMU_ACCEL_COV                                                                                                  \
  {                                                                                                                    \
    0.05, 0.05, 0.05                                                                                                   \
  }

class ahrsBringup : public rclcpp::Node
{
public:
  ahrsBringup();
  ~ahrsBringup();
  void processLoop();
  bool checkCS8(int len);
  bool checkCS16(int len);
  void checkSN(int type);
  void magCalculateYaw(double roll, double pitch, double& magyaw, double magx, double magy, double magz);
  rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter>& parameters);

private:
  OnSetParametersCallbackHandle::SharedPtr parameter_handler_;
  bool if_debug_;
  // sum info
  int sn_lost_ = 0;
  int crc_error_ = 0;
  uint8_t read_sn_ = 0;
  bool frist_sn_;
  int device_type_ = 1;

  // covariance info
  std::vector<double> imu_mag_cov;
  std::vector<double> imu_gyro_cov;
  std::vector<double> imu_accel_cov;

  // serial
  std::unique_ptr<IoContext> owned_ctx_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::string serial_port_;
  int serial_baud_;
  int serial_timeout_;
  // data
  FDILink::imu_frame_read imu_frame_;
  FDILink::ahrs_frame_read ahrs_frame_;
  FDILink::insgps_frame_read insgps_frame_;

  // frame name
  string imu_frame_id_;

  // topic
  string imu_topic_, mag_pose_2d_topic_, imu_topic_trueEast_, mag_topic_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_trueEast_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr mag_pose_pub_;

  // others
  double yaw_offset;
  tf2::Quaternion q_rot;
  double mag_offset_x_;
  double mag_offset_y_;
  double mag_offset_z_;
  double mag_covariance_;

  double frequency_;
  diagnostic_updater::Updater updater_;
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::Imu>> diagnosed_imu_publisher_;
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::MagneticField>> diagnosed_mag_publisher_;
};  // ahrsBringup
}  // namespace FDILink

#endif
