#include <rclcpp/rclcpp.hpp>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <iomanip>
#include <sstream>

class RgbOctomapToPly : public rclcpp::Node
{
public:
  RgbOctomapToPly(const rclcpp::NodeOptions &opts)
  : Node("rgb_octomap_to_ply", opts)
  {
    // Parameter: Pfad zur gespeicherten Octomap (.ot oder .bt)
    input_map_path_   = declare_parameter<std::string>("input_octomap_path", "");
    // Parameter: Ausgabedatei .ply
    output_ply_path_  = declare_parameter<std::string>("output_ply_path", "rgb_map.ply");
    // neue Parameter für Header-Metadaten
    team_name_        = declare_parameter<std::string>("team_name", "MyTeam");
    mission_number_   = declare_parameter<std::string>("mission_number", "1");

    // aktuellen Zeitstempel holen
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time_t);
    std::ostringstream time_stream;
    time_stream << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S");
    start_time_str_ = time_stream.str();

    if (input_map_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'input_octomap_path' ist leer!");
      rclcpp::shutdown();
      return;
    }

    // Octomap laden
    std::unique_ptr<octomap::AbstractOcTree> tree(
      octomap::AbstractOcTree::read(input_map_path_));
    if (!tree) {
      RCLCPP_ERROR(get_logger(), "Fehler beim Laden der Octomap: %s", input_map_path_.c_str());
      rclcpp::shutdown();
      return;
    }

    auto color_tree = dynamic_cast<octomap::ColorOcTree*>(tree.get());
    if (!color_tree) {
      RCLCPP_ERROR(get_logger(), "Datei enthält keine ColorOcTree!");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "RGB Octomap geladen: %zu Knoten, Auflösung %f",
                color_tree->size(), color_tree->getResolution());

    // Zähle besetzte Blätter
    size_t vertex_count = 0;
    for (auto it=color_tree->begin_leafs(), end=color_tree->end_leafs(); it!=end; ++it) {
      if (color_tree->isNodeOccupied(*it)) ++vertex_count;
    }

    // PLY öffnen
    std::ofstream ply(output_ply_path_);
    if (!ply.is_open()) {
      RCLCPP_ERROR(get_logger(), "Konnte PLY nicht erstellen: %s", output_ply_path_.c_str());
      rclcpp::shutdown();
      return;
    }

    // neuer Header
    ply << "ply\n"
        << "format ascii 1.0\n"
        << "comment " << team_name_      << "\n"
        << "comment " << start_time_str_ << "\n"
        << "comment Mission " << mission_number_ << "\n"
        << "element vertex " << vertex_count << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n"
        << "property uchar red\n"
        << "property uchar green\n"
        << "property uchar blue\n"
        << "property float nx\n"
        << "property float ny\n"
        << "property float nz\n"
        << "property float temp\n"
        << "property float confidence\n"
        << "end_header\n";

    // Vertices mit RGB + Normale/Temp/Confidence (Platzhalter)
    for (auto it=color_tree->begin_leafs(), end=color_tree->end_leafs(); it!=end; ++it) {
      if (!color_tree->isNodeOccupied(*it)) continue;
      auto coord = it.getCoordinate();
      auto col   = it->getColor();

      float nx = 0.0f, ny = 0.0f, nz = 1.0f;    // Beispiel-Normale
      float temp = 0.0f;                        // Placeholder
      float confidence = it->getValue();        // Oktomap-Wahrscheinlichkeit als confidence

      ply << coord.x() << " "
          << coord.y() << " "
          << coord.z() << " "
          << int(col.r) << " "
          << int(col.g) << " "
          << int(col.b) << " "
          << nx << " "
          << ny << " "
          << nz << " "
          << temp << " "
          << confidence << "\n";
    }
    ply.close();

    RCLCPP_INFO(get_logger(), "PLY geschrieben nach: %s", output_ply_path_.c_str());
    rclcpp::shutdown();
  }

private:
  std::string input_map_path_, output_ply_path_;
  std::string team_name_, mission_number_, start_time_str_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RgbOctomapToPly)

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // Lädt die Komponente als Standalone-Executable
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<RgbOctomapToPly>(rclcpp::NodeOptions());
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}