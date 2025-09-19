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
    // /// imu data to odom
    // tf2::Quaternion q(
    //     imu.orientation.x,
    //     imu.orientation.y,
    //     imu.orientation.z,
    //     imu.orientation.w);

    // tf2::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);

    // theta_ = yaw; // IMU에서 얻은 yaw 값 업데이트
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
