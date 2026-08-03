#include <yaml-cpp/yaml.h>
#include "sensor_msgs/msg/camera_info.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class OrbCameraInfo : public rclcpp::Node {
public:
  OrbCameraInfo() : Node("orb_camera_info_node")
  {
    camera_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>("/orb_camera/info", 10);
    camera_info_node_ =
      YAML::LoadFile(std::string(PROJECT_PATH) +
                     "/config/Monocular-Inertial/RealSense_D435i.yaml");
    timer_ =
      create_wall_timer(33ms, std::bind(&OrbCameraInfo::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>();
    camera_info->header.frame_id = "base_link";
    camera_info->header.stamp = get_clock()->now();

    camera_info->width = camera_info_node_["Camera.width"].as<int>();
    camera_info->height = camera_info_node_["Camera.height"].as<int>();
    camera_info->distortion_model = "plumb_bob";
    camera_info->k = {camera_info_node_["Camera1.fx"].as<double>(), 0.0, camera_info_node_["Camera1.cx"].as<double>(),
                      0.0, camera_info_node_["Camera1.fy"].as<double>(), camera_info_node_["Camera1.cy"].as<double>(),
                      0.0, 0.0, 1.0};
    camera_info->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info->p = {camera_info_node_["Camera1.fx"].as<double>(), 0.0, camera_info_node_["Camera1.cx"].as<double>(), 0.0,
                      0.0, camera_info_node_["Camera1.fy"].as<double>(), camera_info_node_["Camera1.cy"].as<double>(), 0.0,
                      0.0, 0.0, 1.0, 0.0};

    camera_info_publisher_->publish(*camera_info);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
    camera_info_publisher_;
  YAML::Node camera_info_node_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbCameraInfo>());
  rclcpp::shutdown();
  return 0;
}
