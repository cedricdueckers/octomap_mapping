#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace octomap_server
{
using octomap::AbstractOcTree;
using octomap::ColorOcTree;
using octomap_msgs::srv::GetOctomap;

class RgbOctomapSaver : public rclcpp::Node
{
public:
  explicit RgbOctomapSaver(const rclcpp::NodeOptions & node_options);

private:
  void saveRgbOctomap(const std::string & map_name, std::shared_ptr<ColorOcTree> color_octree);
};

RgbOctomapSaver::RgbOctomapSaver(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("rgb_octomap_saver", node_options)
{
  using namespace std::chrono_literals;

  const auto map_name = declare_parameter("octomap_path", "");
  if (map_name.length() < 4) {
    RCLCPP_ERROR_STREAM(get_logger(), "Invalid file name or extension: " << map_name);
    rclcpp::shutdown();
    return;
  }

  auto client = create_client<GetOctomap>("color_octomap");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for service...");
  }

  auto request = std::make_shared<GetOctomap::Request>();
  auto response = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(
        get_node_base_interface(),
        response) == rclcpp::FutureReturnCode::SUCCESS)
  {
    std::unique_ptr<AbstractOcTree> tree{octomap_msgs::msgToMap(response.get()->map)};
    if (tree) {
      auto color_octree = std::unique_ptr<ColorOcTree>(dynamic_cast<ColorOcTree *>(tree.release()));
      if (color_octree) {
        RCLCPP_INFO(
          get_logger(),
          "RGB Map received (%zu nodes, %f m res), saving to %s",
          color_octree->size(), color_octree->getResolution(), map_name.c_str()
        );
        saveRgbOctomap(map_name, std::move(color_octree));
      } else {
        RCLCPP_ERROR(get_logger(), "Received map is not a ColorOcTree.");
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Error creating octree from received message.");
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Problem while waiting for response.");
  }

  rclcpp::shutdown();
}

void RgbOctomapSaver::saveRgbOctomap(const std::string & map_name, std::shared_ptr<ColorOcTree> color_octree)
{
  std::string suffix = map_name.substr(map_name.length() - 3, 3);
  if (suffix == ".bt") {  // write to binary file
    if (!color_octree->writeBinary(map_name)) {
      RCLCPP_ERROR(get_logger(), "Error writing to file %s", map_name.c_str());
    }
  } else if (suffix == ".ot") {  // write to full .ot file
    if (!color_octree->write(map_name)) {
      RCLCPP_ERROR(get_logger(), "Error writing to file %s", map_name.c_str());
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown file extension, must be either .bt or .ot");
  }
}

}  // namespace octomap_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::RgbOctomapSaver)