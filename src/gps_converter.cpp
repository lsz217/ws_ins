#include "ins_processing/gps_converter.hpp"
#include <cmath>
#include <iostream>
#include "Eigen/Dense"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace ins_processing
{

GpsConverter::GpsConverter(const rclcpp::NodeOptions & options)
: Node("gps_converter", options)
{
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gps/pose", 10);
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/gps/path", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 10, std::bind(&GpsConverter::gps_callback, this, std::placeholders::_1));

    // 默认原点坐标
    lat0_ = this->declare_parameter("lat0", 32.65367390);
    lon0_ = this->declare_parameter("lon0", 110.73012810);
    alt0_ = this->declare_parameter("alt0", 292.097);

    RCLCPP_INFO(this->get_logger(), "GpsConverter node started");
}

void GpsConverter::geodeticToECEF(double lat, double lon, double h, double& x, double& y, double& z)
{
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double e2 = 2 * f - f * f;

    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double N = a / std::sqrt(1 - e2 * std::sin(lat_rad) * std::sin(lat_rad));

    x = (N + h) * std::cos(lat_rad) * std::cos(lon_rad);
    y = (N + h) * std::cos(lat_rad) * std::sin(lon_rad);
    z = ((1 - e2) * N + h) * std::sin(lat_rad);
}

Eigen::Matrix3d GpsConverter::getRotationMatrix(double lat0, double lon0)
{
    double sin_lat = std::sin(lat0 * M_PI / 180.0);
    double cos_lat = std::cos(lat0 * M_PI / 180.0);
    double sin_lon = std::sin(lon0 * M_PI / 180.0);
    double cos_lon = std::cos(lon0 * M_PI / 180.0);
    Eigen::Matrix3d R;
    R << -sin_lon, cos_lon, 0,
         -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
          cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
    return R;
}

void GpsConverter::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "[GpsConverter] GPS Callback triggered!");

    double lat0 = lat0_; // 原点纬度
    double lon0 = lon0_; // 原点经度
    double alt0 = alt0_;  // 原点海拔

    // 经纬度 → ECEF
    double x, y, z;
    geodeticToECEF(msg->latitude, msg->longitude, msg->altitude, x, y, z);
    double x0, y0, z0;
    geodeticToECEF(lat0, lon0, alt0, x0, y0, z0);

    // 差值
    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;

    // ECEF → ENU
    Eigen::Matrix3d R = getRotationMatrix(lat0, lon0);
    Eigen::Vector3d ecef_diff(dx, dy, dz);
    Eigen::Vector3d enu = R * ecef_diff;

    // 缩放
    double scale_factor = 100.0;
    enu[0] /= scale_factor;
    enu[1] /= scale_factor;
    enu[2] /= scale_factor;

    // 构造 PoseStamped 消息
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "enu";
    pose_msg.pose.position.x = enu[0];
    pose_msg.pose.position.y = enu[1];
    pose_msg.pose.position.z = enu[2];
    pose_msg.pose.orientation.w = 1.0;

    // 发布 pose
    publisher_->publish(pose_msg);

    // 保存并发布 path
    path_msg_.poses.push_back(pose_msg);
    publish_path();
}

void GpsConverter::publish_path()
{
    path_msg_.header.stamp = this->now();
    path_msg_.header.frame_id = "enu";
    path_publisher_->publish(path_msg_);
}

}  // namespace ins_processing

// 注册组件节点
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ins_processing::GpsConverter)


