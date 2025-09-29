#include <functional>

#include <sync_imu_lidar/sync_imu_lidar.hpp>

namespace sync_imu_lidar
{
  /// class SyncImuLidar ///

  SyncImuLidar::SyncImuLidar() : Node("sync_imu_lidar")
  {
    this->setup();
  }

  void SyncImuLidar::setup()
  {
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/sync_imu", 10);
    imu_heading_pub = this->create_publisher<std_msgs::msg::Float32>("/imu/heading", 10);
    velo_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sync_velo", 10);
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&SyncImuLidar::imu_callback, this, std::placeholders::_1));
    velo_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 10, std::bind(&SyncImuLidar::velo_callback, this, std::placeholders::_1));

    timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SyncImuLidar::pub_data, this));
  }

  void SyncImuLidar::imu_callback(const sensor_msgs::msg::Imu msg)
  {
    imu = msg;
    tf2::Quaternion q;
    tf2::fromMsg(msg.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    if (!yaw_zero_set_)
    {
      yaw_zero_rad_ = yaw;
      yaw_zero_set_ = true;
    }
    double rel = normalizeRad(yaw - yaw_zero_rad_);
    double heading_deg = -(rel * 180.0 / M_PI  * 180.0 / M_PI); //right + left -
    //왜인진모르겠는데 두번해야 deg가 됨 나중에 이유찾을것
    if (heading_deg > 180.0)  // 혹시ㄹ라서한번만더함
      heading_deg -= 360.0;
    if (heading_deg < -180.0)
      heading_deg += 360.0;
    imu_heading.data = static_cast<float>(heading_deg);
  }

  void SyncImuLidar::velo_callback(const sensor_msgs::msg::PointCloud2 msg)
  {
    velo = msg;
    pub_data();
  }

  void SyncImuLidar::pub_data()
  {
    auto time_stamp = this->get_clock()->now();
    imu.header.stamp = time_stamp;
    velo.header.stamp = time_stamp;
    imu_heading_pub->publish(imu_heading);
    imu_pub->publish(imu);
    velo_pub->publish(velo);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<sync_imu_lidar::SyncImuLidar>();
  rclcpp::Rate rate(10); // 10 Hz

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
