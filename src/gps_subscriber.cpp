#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace ins_processing
{

class GpsSubscriber : public rclcpp::Node
{
public:
  GpsSubscriber(const rclcpp::NodeOptions & options)
  : Node("gps_subscriber", options)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10,
      std::bind(&GpsSubscriber::gps_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "GpsSubscriber node started, waiting for messages...");
  }

private:
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    double latitude  = msg->latitude;
    double longitude = msg->longitude;
    double altitude  = msg->altitude;

    RCLCPP_INFO(this->get_logger(),
      "Received GPS → Lat: %.8f, Lon: %.8f, Alt: %.2f",
      latitude, longitude, altitude);
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

}  // namespace ins_processing

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ins_processing::GpsSubscriber)
