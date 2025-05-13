#ifndef INS_PROCESSING_GPS_CONVERTER_HPP
#define INS_PROCESSING_GPS_CONVERTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <Eigen/Dense>

namespace ins_processing
{

class GpsConverter : public rclcpp::Node
{
public:
    explicit GpsConverter(const rclcpp::NodeOptions & options);
    
private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void publish_path();
    Eigen::Matrix3d getRotationMatrix(double lat0, double lon0);
    void geodeticToECEF(double lat, double lon, double h, double& x, double& y, double& z);
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_msg_;
    
    double lat0_, lon0_, alt0_;
};

}  // namespace ins_processing

#endif  // INS_PROCESSING_GPS_CONVERTER_HPP








