#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <iostream>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace sync_imu_lidar
{
    class SyncImuLidar : public rclcpp::Node
    {

    public:
        SyncImuLidar();

    private:
        void setup();
        void imu_callback(const sensor_msgs::msg::Imu msg);
        void velo_callback(const sensor_msgs::msg::PointCloud2 msg);
        void pub_data();

    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velo_pub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velo_sub;

        sensor_msgs::msg::PointCloud2 velo;
        sensor_msgs::msg::Imu imu;
        rclcpp::TimerBase::SharedPtr timer;
        
    };
}
