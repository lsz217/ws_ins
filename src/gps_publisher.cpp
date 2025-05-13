#include "ins_processing/gps_publisher.hpp"
#include <chrono>
#include <fstream>
#include <sstream>

namespace ins_processing
{

GpsPublisher::GpsPublisher()
: Node("gps_publisher"), current_index_(0)
{
    // 默认路径
    std::string file_path = "/home/lsz/下载/gps.txt";
    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GpsPublisher::publish_gps_data, this));

    read_gps_data_from_file(file_path);

    RCLCPP_INFO(this->get_logger(), "GpsPublisher node started with default file: %s", file_path.c_str());
}

GpsPublisher::GpsPublisher(const rclcpp::NodeOptions & options)
: Node("gps_publisher", options), current_index_(0)
{
    this->declare_parameter<std::string>("file_path", "/home/lsz/下载/gps.txt");
    std::string file_path = this->get_parameter("file_path").as_string();

    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GpsPublisher::publish_gps_data, this));

    read_gps_data_from_file(file_path);

    RCLCPP_INFO(this->get_logger(), "GpsPublisher node started with file: %s", file_path.c_str());
}

void GpsPublisher::read_gps_data_from_file(const std::string &file_path)
{
    std::ifstream file(file_path);
    std::string line;

    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
        return;
    }

    while (std::getline(file, line)) {
        double lat, lon, alt;
        if (sscanf(line.c_str(), "lat = %lf\tlon = %lf\talt = %lf", &lat, &lon, &alt) == 3) {
            gps_data_.emplace_back(lat, lon, alt);
            RCLCPP_INFO(this->get_logger(), "Read GPS → Lat: %.8f, Lon: %.8f, Alt: %.2f", lat, lon, alt);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to parse line: %s", line.c_str());
        }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu GPS entries.", gps_data_.size());
}

void GpsPublisher::publish_gps_data()
{
    if (gps_data_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No GPS data to publish!");
        return;
    }

    auto msg = sensor_msgs::msg::NavSatFix();
    auto &[lat, lon, alt] = gps_data_[current_index_];

    msg.latitude = lat;
    msg.longitude = lon;
    msg.altitude = alt;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "gps_link";

    RCLCPP_DEBUG(this->get_logger(), "Publishing → Lat: %.8f, Lon: %.8f, Alt: %.2f", lat, lon, alt);
    publisher_->publish(msg);

    current_index_ = (current_index_ + 1) % gps_data_.size();
}

}  // namespace ins_processing

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ins_processing::GpsPublisher)
