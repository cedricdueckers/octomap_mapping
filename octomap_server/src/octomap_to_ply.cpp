#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <ctime>
#include <iomanip>
#include <sstream>

#include <chrono>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

namespace octomap_server
{
using octomap::AbstractOcTree;
using octomap::OcTree;

class OctomapToPly : public rclcpp::Node
{
public:
  explicit OctomapToPly(const rclcpp::NodeOptions & node_options);
};

// Function to calculate color based on Z-coordinate
void calculateColor(float z, unsigned char &red, unsigned char &green, unsigned char &blue) {
  // Normalize Z value based on min_z and max_z
  //float normalized_z = (z - 0) / 0.5f;  // Assuming min_z = 0 and max_z = 1.0 for this example

  // Map normalized Z to a gradient (red to blue)
  red = static_cast<unsigned char>((1-z) * 255);
  green = 0;  // No green in this gradient
  blue = static_cast<unsigned char>(z * 255);


}

OctomapToPly::OctomapToPly(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("octomap_to_ply", node_options)
{
  const auto input_map_path = declare_parameter<std::string>("input_octomap_path", "");
  const auto team_name = "Maskor";
  const auto mission_number = "1";

  // Get current time for filename
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm = *std::localtime(&now_time_t);

  std::ostringstream time_stream;
  time_stream << std::put_time(&now_tm, "%H-%M-%S");

  const std::string output_ply_path = std::string("RoboCup2025-") + team_name + "-Mission" + mission_number + "-" +
                                      time_stream.str() + "-map.ply";

  if (input_map_path.empty()) {
    RCLCPP_ERROR(get_logger(), "Input path is missing!");
    rclcpp::shutdown();
    return;
  }

  // Load the Octomap
  std::unique_ptr<AbstractOcTree> tree(octomap::AbstractOcTree::read(input_map_path));
  if (!tree) {
    RCLCPP_ERROR(get_logger(), "Failed to read Octomap from file: %s", input_map_path.c_str());
    rclcpp::shutdown();
    return;
  }

  OcTree* octree = dynamic_cast<OcTree*>(tree.get());
  if (!octree) {
    RCLCPP_ERROR(get_logger(), "Input file is not an OcTree!");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Successfully loaded Octomap (%zu nodes, %f m resolution).",
    octree->size(), octree->getResolution()
  );

  // Count the number of occupied nodes
  size_t vertex_count = 0;
  for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
    if (octree->isNodeOccupied(*it)) {
      ++vertex_count;
    }
  }

  // Open file for writing .ply
  std::ofstream ply_file(output_ply_path);
  if (!ply_file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file for writing: %s", output_ply_path.c_str());
    rclcpp::shutdown();
    return;
  }

  // Write .ply header
  ply_file << "ply\n";
  ply_file << "format ascii 1.0\n";
  ply_file << "comment " << team_name << "\n";
  ply_file << "comment " << time_stream.str() << "\n";
  ply_file << "comment Mission " << mission_number << "\n";
  ply_file << "element vertex " << vertex_count << "\n";
  ply_file << "property float x\n";
  ply_file << "property float y\n";
  ply_file << "property float z\n";
  ply_file << "property uchar red\n";
  ply_file << "property uchar green\n";
  ply_file << "property uchar blue\n";
  ply_file << "property float nx\n";
  ply_file << "property float ny\n";
  ply_file << "property float nz\n";
  ply_file << "property float temp\n";
  ply_file << "property float confidence\n";
  ply_file << "end_header\n";

  // Write the vertices
  for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
    if (octree->isNodeOccupied(*it)) {
      const auto& coord = it.getCoordinate();

      // Calculate color based on Z-coordinate
      unsigned char red, green, blue;
      calculateColor(coord.z(), red, green, blue);
      RCLCPP_INFO(get_logger(), "Z-coordinate: %f", coord.z());

      // Example values for additional properties
      float nx = 1.0f;  // Placeholder for normal x
      float ny = 1.0f;  // Placeholder for normal y
      float nz = 1.0f;  // Placeholder for normal z
      float temp = 0.0f;  // Placeholder for temperature
      float confidence = 1.0f;  // Placeholder for confidence

      ply_file << coord.x() << " " << coord.y() << " " << coord.z() << " "
               << static_cast<int>(red) << " " << static_cast<int>(green) << " " << static_cast<int>(blue) << " "
               << nx << " " << ny << " " << nz << " "
               << temp << " " << confidence << "\n";
    }
  }

  ply_file.close();
  RCLCPP_INFO(get_logger(), "Successfully wrote PLY file to: %s", output_ply_path.c_str());

  rclcpp::shutdown();
}
}  // namespace octomap_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::OctomapToPly)

// ros2 run <your_package> octomap_to_ply --ros-args -p input_octomap_path:=input.ot -p team_name:=MyTeam -p mission_number:=2