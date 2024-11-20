#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "hik_camera/hik_camera.hpp"



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto hikcamera_node = std::make_shared<camera::CameraNode>("hik_camera", options);

  exec.add_node(hikcamera_node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}

