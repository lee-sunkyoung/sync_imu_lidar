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
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace sync_imu_lidar
{
    static inline double normalizeRad(double a)
    {
        return std::atan2(std::sin(a), std::cos(a)); // [-pi, pi]
    }

    class SyncImuLidar : public rclcpp::Node
    {

    public:
        SyncImuLidar();

    private:
        void setup();
        void pub_imu_heading();
        void imu_callback(const sensor_msgs::msg::Imu msg);
        void velo_callback(const sensor_msgs::msg::PointCloud2 msg);
        void pub_data();

    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_heading_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velo_pub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velo_sub;

        bool yaw_zero_set_ = false;
        double yaw_zero_rad_ = 0.0;
        std_msgs::msg::Float32 imu_heading;
        sensor_msgs::msg::PointCloud2 velo;
        sensor_msgs::msg::Imu imu;
        rclcpp::TimerBase::SharedPtr timer;
    };
}
