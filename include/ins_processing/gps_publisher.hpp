#ifndef GPS_PUBLISHER_HPP_
#define GPS_PUBLISHER_HPP_

#include <vector>
#include <tuple>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace ins_processing
{

class GpsPublisher : public rclcpp::Node
{
public:
    GpsPublisher();  // 无参构造函数（pluginlib 需要）
    explicit GpsPublisher(const rclcpp::NodeOptions & options);  // NodeOptions 构造函数

private:
    void read_gps_data_from_file(const std::string &file_path);
    void publish_gps_data();

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::tuple<double, double, double>> gps_data_;
    size_t current_index_;
};

}  // namespace ins_processing

#endif  // GPS_PUBLISHER_HPP_



