#include <ahrs_driver.h>
#include <Eigen/Eigen>
#include <memory>
#include <rclcpp/executors.hpp>
#include "tf2/transform_datatypes.h"
namespace FDILink
{
ahrsBringup::ahrsBringup()
  : rclcpp::Node("ahrs_bringup")
  , frist_sn_(false)
  , serial_timeout_(20)
  , mag_offset_x_(0)
  , mag_offset_y_(0)
  , mag_offset_z_(0)
  , mag_covariance_(0)
  , owned_ctx_(new IoContext(2))
  , serial_driver_(new drivers::serial_driver::SerialDriver(*owned_ctx_))
  , updater_(this)
{
  // topic_name & frame_id
  this->declare_parameter("debug", false);
  this->declare_parameter("device_type", 1);  // default: single imu
  this->declare_parameter("imu_topic", "imu");
  this->declare_parameter("imu_frame", "imu");
  this->declare_parameter("mag_pose_2d_topic", "mag_pose_2d");
  this->declare_parameter("imu_topic_trueEast", "imu_trueEast");
  this->declare_parameter("mag_topic", "magnetic_field");
  this->declare_parameter("yaw_offset", -2.094);
  this->declare_parameter("diagnostic_tolerance", 0.1);
  this->get_parameter("debug", if_debug_);
  this->get_parameter("device_type", device_type_);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("imu_frame", imu_frame_id_);
  this->get_parameter("mag_pose_2d_topic", mag_pose_2d_topic_);
  this->get_parameter("imu_topic_trueEast", imu_topic_trueEast_);
  this->get_parameter("mag_topic", mag_topic_);
  this->get_parameter("yaw_offset", yaw_offset);
  q_rot.setRPY(0, 0, yaw_offset);
  // sensor covariance setting

  this->declare_parameter("imu_mag_covVec",
                          std::vector<double> IMU_MAG_COV);  // default: sample covariances from header
  this->declare_parameter("imu_gyro_covVec",
                          std::vector<double> IMU_GYRO_COV);  // default: sample covariances from header
  this->declare_parameter("imu_accel_covVec",
                          std::vector<double> IMU_ACCEL_COV);  // default: sample covariances from header
  this->get_parameter("imu_mag_covVec", imu_mag_cov);
  this->get_parameter("imu_gyro_covVec", imu_gyro_cov);
  this->get_parameter("imu_accel_covVec", imu_accel_cov);
  this->declare_parameter("mag_bias_x", 0.0);
  this->declare_parameter("mag_bias_y", 0.0);
  this->declare_parameter("mag_bias_z", 0.0);
  this->declare_parameter("mag_covariance", 0.0);
  this->get_parameter("mag_bias_x", mag_offset_x_);
  this->get_parameter("mag_bias_y", mag_offset_y_);
  this->get_parameter("mag_bias_z", mag_offset_z_);
  this->get_parameter("mag_covariance", mag_covariance_);
  parameter_handler_ =
      this->add_on_set_parameters_callback(std::bind(&ahrsBringup::parameterCallback, this, std::placeholders::_1));
  // serial
  this->declare_parameter("port", "/dev/ttyUSB1");
  this->declare_parameter("baud", 921600);
  this->get_parameter("port", serial_port_);
  this->get_parameter("baud", serial_baud_);
  // publisher
  auto imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
  imu_trueEast_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_trueEast_, 10);
  mag_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>(mag_pose_2d_topic_, 10);
  auto mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_, 10);

  updater_.setHardwareID("ahrs");
  auto tolerance = this->get_parameter("diagnostic_tolerance").as_double();
  frequency_ = 100.0;

  diagnosed_imu_publisher_ = std::make_shared<diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::Imu>>(
    imu_pub, updater_, diagnostic_updater::FrequencyStatusParam(&frequency_, &frequency_, tolerance, 10),
    diagnostic_updater::TimeStampStatusParam());
  diagnosed_mag_publisher_ = std::make_shared<diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::MagneticField>>(
    mag_pub, updater_, diagnostic_updater::FrequencyStatusParam(&frequency_, &frequency_, tolerance, 10),
    diagnostic_updater::TimeStampStatusParam());
  // setp up serial
  try
  {
    const auto fc = drivers::serial_driver::FlowControl::NONE;
    const auto pt = drivers::serial_driver::Parity::NONE;
    const auto sb = drivers::serial_driver::StopBits::ONE;
    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(serial_baud_, fc, pt, sb);
    serial_driver_->init_port(serial_port_, *device_config_);
    serial_driver_->port()->open();
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open port ");
    exit(0);
  }
  if (serial_driver_->port() && serial_driver_->port()->is_open())
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Serial Port initialized");
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to initial Serial port ");
    exit(0);
  }
  processLoop();
}

ahrsBringup::~ahrsBringup()
{
  if (serial_driver_->port() && serial_driver_->port()->is_open())
    serial_driver_->port()->close();
}

rcl_interfaces::msg::SetParametersResult ahrsBringup::parameterCallback(const std::vector<rclcpp::Parameter>& params)
{
  auto results = rcl_interfaces::msg::SetParametersResult();

  for (const auto& param : params)
  {
    if (param.get_name() == "mag_bias_x")
    {
      mag_offset_x_ = param.as_double();
    }
    else if (param.get_name() == "mag_bias_y")
    {
      mag_offset_y_ = param.as_double();
    }
    else if (param.get_name() == "mag_bias_z")
    {
      mag_offset_z_ = param.as_double();
    }
    else if (param.get_name() == "mag_covariance")
    {
      mag_covariance_ = param.as_double();
    }
  }

  results.successful = true;
  return results;
}

void ahrsBringup::processLoop()
{
  RCLCPP_INFO(this->get_logger(), "ahrsBringup::processLoop: start");
  while (rclcpp::ok())
  {
    if (!serial_driver_->port()->is_open())
    {
      RCLCPP_WARN(this->get_logger(), "serial unopen");
    }
    // check head start
    std::vector<uint8_t> check_head(1);
    size_t head_s = serial_driver_->port()->receive(check_head);
    if (if_debug_)
    {
      if (head_s != 1)
      {
        RCLCPP_ERROR(this->get_logger(), "Read serial port time out! can't read pack head.");
      }
      std::cout << std::endl;
      std::cout << "check_head: " << std::hex << (int)check_head[0] << std::dec << std::endl;
    }
    if (check_head[0] != FRAME_HEAD)
    {
      continue;
    }
    // check head type
    std::vector<uint8_t> head_type(1);
    size_t type_s = serial_driver_->port()->receive(head_type);
    if (if_debug_)
    {
      std::cout << "head_type:  " << std::hex << (int)head_type[0] << std::dec << std::endl;
    }
    if (head_type[0] != TYPE_IMU && head_type[0] != TYPE_AHRS && head_type[0] != TYPE_INSGPS && head_type[0] != 0x50 &&
        head_type[0] != TYPE_GROUND)
    {
      RCLCPP_WARN(this->get_logger(), "head_type error: %02X", head_type[0]);
      continue;
    }
    // check head length
    std::vector<uint8_t> check_len(1);
    size_t len_s = serial_driver_->port()->receive(check_len);
    if (if_debug_)
    {
      std::cout << "check_len: " << std::dec << (int)check_len[0] << std::endl;
    }
    if (head_type[0] == TYPE_IMU && check_len[0] != IMU_LEN)
    {
      RCLCPP_WARN(this->get_logger(), "head_len error (imu)");
      continue;
    }
    else if (head_type[0] == TYPE_AHRS && check_len[0] != AHRS_LEN)
    {
      RCLCPP_WARN(this->get_logger(), "head_len error (ahrs)");
      continue;
    }
    else if (head_type[0] == TYPE_INSGPS && check_len[0] != INSGPS_LEN)
    {
      RCLCPP_WARN(this->get_logger(), "head_len error (insgps)");
      continue;
    }
    else if (head_type[0] == TYPE_GROUND || head_type[0] == 0x50)  // 未知数据，防止记录失败
    {
      std::vector<uint8_t> ground_sn(1);
      size_t ground_sn_s = serial_driver_->port()->receive(ground_sn);
      if (++read_sn_ != ground_sn[0])
      {
        if (ground_sn[0] < read_sn_)
        {
          if (if_debug_)
          {
            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
          }
          sn_lost_ += 256 - (int)(read_sn_ - ground_sn[0]);
          read_sn_ = ground_sn[0];
          // continue;
        }
        else
        {
          if (if_debug_)
          {
            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
          }
          sn_lost_ += (int)(ground_sn[0] - read_sn_);
          read_sn_ = ground_sn[0];
          // continue;
        }
      }
      std::vector<uint8_t> ground_ignore(check_len[0] + 4);
      size_t ground_ignore_s = serial_driver_->port()->receive(ground_ignore);
      continue;
    }
    // read head sn
    std::vector<uint8_t> check_sn(1);
    size_t sn_s = serial_driver_->port()->receive(check_sn);
    std::vector<uint8_t> head_crc8(1);
    size_t crc8_s = serial_driver_->port()->receive(head_crc8);
    std::vector<uint8_t> head_crc16_H(1);
    size_t crc16_H_s = serial_driver_->port()->receive(head_crc16_H);
    std::vector<uint8_t> head_crc16_L(1);
    size_t crc16_L_s = serial_driver_->port()->receive(head_crc16_L);
    if (if_debug_)
    {
      std::cout << "check_sn: " << std::hex << (int)check_sn[0] << std::dec << std::endl;
      std::cout << "head_crc8: " << std::hex << (int)head_crc8[0] << std::dec << std::endl;
      std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl;
      std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl;
    }
    // put header & check crc8 & count sn lost
    if (head_type[0] == TYPE_IMU)
    {
      imu_frame_.frame.header.header_start = check_head[0];
      imu_frame_.frame.header.data_type = head_type[0];
      imu_frame_.frame.header.data_size = check_len[0];
      imu_frame_.frame.header.serial_num = check_sn[0];
      imu_frame_.frame.header.header_crc8 = head_crc8[0];
      imu_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      imu_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(imu_frame_.read_buf.frame_header, 4);
      if (CRC8 != imu_frame_.frame.header.header_crc8)
      {
        RCLCPP_WARN(this->get_logger(), "header_crc8 error");
        continue;
      }
      if (!frist_sn_)
      {
        read_sn_ = imu_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      // check sn
      ahrsBringup::checkSN(TYPE_IMU);
    }
    else if (head_type[0] == TYPE_AHRS)
    {
      ahrs_frame_.frame.header.header_start = check_head[0];
      ahrs_frame_.frame.header.data_type = head_type[0];
      ahrs_frame_.frame.header.data_size = check_len[0];
      ahrs_frame_.frame.header.serial_num = check_sn[0];
      ahrs_frame_.frame.header.header_crc8 = head_crc8[0];
      ahrs_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      ahrs_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(ahrs_frame_.read_buf.frame_header, 4);
      if (CRC8 != ahrs_frame_.frame.header.header_crc8)
      {
        RCLCPP_WARN(this->get_logger(), "header_crc8 error");
        continue;
      }
      if (!frist_sn_)
      {
        read_sn_ = ahrs_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      // check sn
      ahrsBringup::checkSN(TYPE_AHRS);
    }
    else if (head_type[0] == TYPE_INSGPS)
    {
      insgps_frame_.frame.header.header_start = check_head[0];
      insgps_frame_.frame.header.data_type = head_type[0];
      insgps_frame_.frame.header.data_size = check_len[0];
      insgps_frame_.frame.header.serial_num = check_sn[0];
      insgps_frame_.frame.header.header_crc8 = head_crc8[0];
      insgps_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      insgps_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(insgps_frame_.read_buf.frame_header, 4);
      if (CRC8 != insgps_frame_.frame.header.header_crc8)
      {
        RCLCPP_WARN(this->get_logger(), "header_crc8 error");
        continue;
      }
      else if (if_debug_)
      {
        std::cout << "header_crc8 matched." << std::endl;
      }
      ahrsBringup::checkSN(TYPE_INSGPS);
    }
    if (head_type[0] == TYPE_IMU)
    {
      uint16_t head_crc16_l = imu_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = imu_frame_.frame.header.header_crc16_h;
      std::vector<uint8_t> msg_buf(IMU_LEN + 1);
      size_t data_s = serial_driver_->port()->receive(msg_buf);
      for (size_t i = 0; i < (IMU_LEN + 1); i++)
      {
        imu_frame_.read_buf.read_msg[i] = msg_buf[i];
      }
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      // if (if_debug_){
      //   for (size_t i = 0; i < (IMU_LEN + 1); i++)
      //   {
      //     std::cout << std::hex << (int)imu_frame_.read_buf.read_msg[i] << " ";
      //   }
      //   std::cout << std::dec << std::endl;
      // }
      uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN);
      if (if_debug_)
      {
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      if (head_crc16 != CRC16)
      {
        RCLCPP_WARN(this->get_logger(), "check crc16 faild(imu).");
        continue;
      }
      else if (imu_frame_.frame.frame_end != FRAME_END)
      {
        RCLCPP_WARN(this->get_logger(), "check frame end.");
        continue;
      }
    }
    else if (head_type[0] == TYPE_AHRS)
    {
      uint16_t head_crc16_l = ahrs_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = ahrs_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      std::vector<uint8_t> msg_buf(AHRS_LEN + 1);
      size_t data_s = serial_driver_->port()->receive(msg_buf);
      for (size_t i = 0; i < (IMU_LEN + 1); i++)
      {
        ahrs_frame_.read_buf.read_msg[i] = msg_buf[i];
      }
      // if (if_debug_){
      //   for (size_t i = 0; i < (AHRS_LEN + 1); i++)
      //   {
      //     std::cout << std::hex << (int)ahrs_frame_.read_buf.read_msg[i] << " ";
      //   }
      //   std::cout << std::dec << std::endl;
      // }
      uint16_t CRC16 = CRC16_Table(ahrs_frame_.frame.data.data_buff, AHRS_LEN);
      if (if_debug_)
      {
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      if (head_crc16 != CRC16)
      {
        RCLCPP_WARN(this->get_logger(), "check crc16 faild(ahrs).");
        continue;
      }
      else if (ahrs_frame_.frame.frame_end != FRAME_END)
      {
        RCLCPP_WARN(this->get_logger(), "check frame end.");
        continue;
      }
    }
    else if (head_type[0] == TYPE_INSGPS)
    {
      uint16_t head_crc16 =
          insgps_frame_.frame.header.header_crc16_l + ((uint16_t)insgps_frame_.frame.header.header_crc16_h << 8);
      std::vector<uint8_t> msg_buf(INSGPS_LEN + 1);
      size_t data_s = serial_driver_->port()->receive(msg_buf);
      for (size_t i = 0; i < (IMU_LEN + 1); i++)
      {
        insgps_frame_.read_buf.read_msg[i] = msg_buf[i];
      }
      uint16_t CRC16 = CRC16_Table(insgps_frame_.frame.data.data_buff, INSGPS_LEN);
      if (head_crc16 != CRC16)
      {
        RCLCPP_WARN(this->get_logger(), "check crc16 faild(insgps).");
        continue;
      }
      else if (insgps_frame_.frame.frame_end != FRAME_END)
      {
        RCLCPP_WARN(this->get_logger(), "check frame end.");
        continue;
      }
    }
    // publish magyaw topic
    if (head_type[0] == TYPE_AHRS)
    {
      // publish imu topic
      sensor_msgs::msg::Imu imu_data, imu_trueEast_data;
      imu_data.header.stamp = this->get_clock()->now();
      imu_data.header.frame_id = imu_frame_id_;
      Eigen::Quaterniond q_ahrs(ahrs_frame_.frame.data.data_pack.Qw, ahrs_frame_.frame.data.data_pack.Qx,
                                ahrs_frame_.frame.data.data_pack.Qy, ahrs_frame_.frame.data.data_pack.Qz);
      Eigen::Quaterniond q_r = Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond q_rr = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond q_z = Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond q_xiao_rr = Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX());
      if (device_type_ == 0)  // 未经变换的原始数据
      {
        imu_data.orientation.w = ahrs_frame_.frame.data.data_pack.Qw;
        imu_data.orientation.x = ahrs_frame_.frame.data.data_pack.Qx;
        imu_data.orientation.y = ahrs_frame_.frame.data.data_pack.Qy;
        imu_data.orientation.z = ahrs_frame_.frame.data.data_pack.Qz;
        imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
        imu_data.angular_velocity.y = ahrs_frame_.frame.data.data_pack.PitchSpeed;
        imu_data.angular_velocity.z = ahrs_frame_.frame.data.data_pack.HeadingSpeed;
        imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
        imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
        imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;
      }
      else if (device_type_ == 1)  // imu单品ROS标准下的坐标变换
      {
        Eigen::Quaterniond q_out = q_z * q_rr * q_ahrs;
        imu_data.orientation.w = q_out.w();
        imu_data.orientation.x = q_out.x();
        imu_data.orientation.y = q_out.y();
        imu_data.orientation.z = q_out.z();
        imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
        imu_data.angular_velocity.y = ahrs_frame_.frame.data.data_pack.PitchSpeed;
        imu_data.angular_velocity.z = ahrs_frame_.frame.data.data_pack.HeadingSpeed;
        imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
        imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
        imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;
      }
      imu_data.orientation_covariance[0] = imu_mag_cov[0];
      imu_data.orientation_covariance[4] = imu_mag_cov[1];
      imu_data.orientation_covariance[8] = imu_mag_cov[2];
      imu_data.angular_velocity_covariance[0] = imu_gyro_cov[0];
      imu_data.angular_velocity_covariance[4] = imu_gyro_cov[1];
      imu_data.angular_velocity_covariance[8] = imu_gyro_cov[2];
      imu_data.linear_acceleration_covariance[0] = imu_accel_cov[0];
      imu_data.linear_acceleration_covariance[4] = imu_accel_cov[1];
      imu_data.linear_acceleration_covariance[8] = imu_accel_cov[2];
      diagnosed_imu_publisher_->publish(imu_data);
      // true East heading publish ----
      tf2::Quaternion q_new;
      tf2::Quaternion q_orig(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z,
                             imu_data.orientation.w);
      q_new = q_rot * q_orig;
      q_new.normalize();

      imu_trueEast_data.orientation.x = q_new.x();
      imu_trueEast_data.orientation.y = q_new.y();
      imu_trueEast_data.orientation.z = q_new.z();
      imu_trueEast_data.orientation.w = q_new.w();
      imu_trueEast_data.orientation_covariance = imu_data.orientation_covariance;
      imu_trueEast_data.angular_velocity_covariance = imu_data.angular_velocity_covariance;
      imu_trueEast_data.linear_acceleration_covariance = imu_data.linear_acceleration_covariance;
      imu_trueEast_pub_->publish(imu_trueEast_data);
      // ------------------------------

      Eigen::Quaterniond rpy_q(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y,
                               imu_data.orientation.z);
      geometry_msgs::msg::Pose2D pose_2d;
      double magx, magy, magz, roll, pitch;
      if (device_type_ == 0)
      {  // 未经变换的原始数据//
        magx = imu_frame_.frame.data.data_pack.magnetometer_x;
        magy = imu_frame_.frame.data.data_pack.magnetometer_y;
        magz = imu_frame_.frame.data.data_pack.magnetometer_z;
        roll = ahrs_frame_.frame.data.data_pack.Roll;
        pitch = ahrs_frame_.frame.data.data_pack.Pitch;
      }
      else if (device_type_ == 1)
      {  // 小车以及imu单品ROS标准下的坐标变换//
        magx = imu_frame_.frame.data.data_pack.magnetometer_x;
        magy = imu_frame_.frame.data.data_pack.magnetometer_y;
        magz = imu_frame_.frame.data.data_pack.magnetometer_z;

        Eigen::Vector3d EulerAngle = rpy_q.matrix().eulerAngles(2, 1, 0);
        roll = EulerAngle[2];
        pitch = EulerAngle[1];
      }

      // Convert mG to T
      magx *= 1.0e-7;
      magy *= 1.0e-7;
      magz *= 1.0e-7;
      magx -= mag_offset_x_;
      magy -= mag_offset_y_;
      magz -= mag_offset_z_;

      double magyaw;
      magCalculateYaw(roll, pitch, magyaw, magx, magy, magz);
      pose_2d.theta = magyaw;
      mag_pose_pub_->publish(pose_2d);

      sensor_msgs::msg::MagneticField mag;
      mag.header = imu_data.header;
      mag.magnetic_field.x = magx;
      mag.magnetic_field.y = magy;
      mag.magnetic_field.z = magz;
      std::fill(mag.magnetic_field_covariance.begin(), mag.magnetic_field_covariance.end(), mag_covariance_);
      diagnosed_mag_publisher_->publish(mag);
    }
    rclcpp::spin_some(this->get_node_base_interface());
  }
}

void ahrsBringup::magCalculateYaw(double roll, double pitch, double& magyaw, double magx, double magy, double magz)
{
  double temp1 = magy * cos(roll) + magz * sin(roll);
  double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
  magyaw = atan2(-temp1, temp2);
  if (magyaw < 0)
  {
    magyaw = magyaw + 2 * PI;
  }
  // return magyaw;
}

void ahrsBringup::checkSN(int type)
{
  switch (type)
  {
    case TYPE_IMU:
      if (++read_sn_ != imu_frame_.frame.header.serial_num)
      {
        if (imu_frame_.frame.header.serial_num < read_sn_)
        {
          sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num);
          if (if_debug_)
          {
            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
          }
        }
        else
        {
          sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_);
          if (if_debug_)
          {
            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
          }
        }
      }
      read_sn_ = imu_frame_.frame.header.serial_num;
      break;

    case TYPE_AHRS:
      if (++read_sn_ != ahrs_frame_.frame.header.serial_num)
      {
        if (ahrs_frame_.frame.header.serial_num < read_sn_)
        {
          sn_lost_ += 256 - (int)(read_sn_ - ahrs_frame_.frame.header.serial_num);
          if (if_debug_)
          {
            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
          }
        }
        else
        {
          sn_lost_ += (int)(ahrs_frame_.frame.header.serial_num - read_sn_);
          if (if_debug_)
          {
            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
          }
        }
      }
      read_sn_ = ahrs_frame_.frame.header.serial_num;
      break;

    case TYPE_INSGPS:
      if (++read_sn_ != insgps_frame_.frame.header.serial_num)
      {
        if (insgps_frame_.frame.header.serial_num < read_sn_)
        {
          sn_lost_ += 256 - (int)(read_sn_ - insgps_frame_.frame.header.serial_num);
          if (if_debug_)
          {
            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
          }
        }
        else
        {
          sn_lost_ += (int)(insgps_frame_.frame.header.serial_num - read_sn_);
          if (if_debug_)
          {
            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
          }
        }
      }
      read_sn_ = insgps_frame_.frame.header.serial_num;
      break;

    default:
      break;
  }
}

}  // namespace FDILink

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FDILink::ahrsBringup>();
  rclcpp::shutdown();
}
