#ifndef GPS_SUBSCRIBER_HPP_
#define GPS_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace ins_processing  // 确保使用一致的命名空间
{

class GpsSubscriber : public rclcpp::Node
{
public:
  // 默认构造函数 + NodeOptions 构造函数（推荐做法）
  explicit GpsSubscriber(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

}  // namespace ins_processing

#endif  // GPS_SUBSCRIBER_HPP_




