#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <filesystem>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

struct Box {
  float x_min = std::numeric_limits<float>::infinity();
  float x_max = -std::numeric_limits<float>::infinity();
  float y_min = std::numeric_limits<float>::infinity();
  float y_max = -std::numeric_limits<float>::infinity();
  float z_min = std::numeric_limits<float>::infinity();
  float z_max = -std::numeric_limits<float>::infinity();
};

class Visualize : public rclcpp::Node {
public:
  Visualize() : Node("visualize")
  {
    // declare parameters
    declare_parameter("output_name", "");

    // get parameters
    get_parameter("output_name", output_name_);

    if (!load_clouds()) {
      RCLCPP_ERROR(get_logger(), "Error loading clouds");
      rclcpp::shutdown();
    }

    // define publishers
    full_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("full_cloud", 10);

    // define timer
    timer_ =
      create_wall_timer(1000ms, std::bind(&Visualize::timer_callback, this));
  }

private:
  bool load_clouds()
  {
    std::string output_path =
      std::string(PROJECT_PATH) + "/output/" + output_name_;
    if (!std::filesystem::is_directory(output_path)) {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Directory " << output_path << " does not exist");
      return false;
    }
    std::string cloud_path =
      output_path + "/cloud/" + output_name_ + ".pcd";
    if (pcl::io::loadPCDFile(cloud_path, full_cloud_) == -1) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error loading file " << cloud_path);
      return false;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Loaded full cloud with "
                                       << full_cloud_.size() << " points");
    return true;
  }
  void timer_callback()
  {
    sensor_msgs::msg::PointCloud2 full_cloud_msg;
    pcl::toROSMsg(full_cloud_, full_cloud_msg);
    full_cloud_msg.header.frame_id = "map";
    full_cloud_msg.header.stamp = get_clock()->now();
    full_cloud_publisher_->publish(full_cloud_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    full_cloud_publisher_;

  pcl::PointCloud<pcl::PointXYZ> full_cloud_;
  std::string output_name_;
  std::string cloud_path_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualize>());
  rclcpp::shutdown();
  return 0;
}
