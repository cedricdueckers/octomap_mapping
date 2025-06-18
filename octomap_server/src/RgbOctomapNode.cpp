#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class RgbOctomapNode : public rclcpp::Node {
public:
    RgbOctomapNode() : Node("rgb_octomap_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "Starte RgbOctomapNode...");
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth_registered/points", 10,
            std::bind(&RgbOctomapNode::pointCloudCallback, this, std::placeholders::_1));

        octree_ = std::make_shared<octomap::ColorOcTree>(0.05);  // Auflösung 5cm
        pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);

        RCLCPP_INFO(this->get_logger(), "Abonniere Topic: /camera/depth_registered/points");
        RCLCPP_INFO(this->get_logger(), "Publisher für Topic: octomap erstellt");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::string target_frame = "odom"; // oder "odom", je nach Welt-Frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(
                target_frame, msg->header.frame_id,
                tf2::TimePointZero, std::chrono::milliseconds(100));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF konnte nicht geholt werden: %s", ex.what());
            return;
        }

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_rgb(*msg, "rgb");

        size_t count = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            // NaN & Tiefenbereich filtern
            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            if (z < 0.3 || z > 5.0) continue;  // nur gültiger Bereich

            // Punkt transformieren
            geometry_msgs::msg::PointStamped pt_in, pt_out;
            pt_in.header = msg->header;
            pt_in.point.x = x;
            pt_in.point.y = y;
            pt_in.point.z = z;
            tf2::doTransform(pt_in, pt_out, transform_stamped);

            // RGB aus float32 extrahieren (standard PCL-Format)
            uint32_t rgb_val = *reinterpret_cast<const uint32_t*>(&(*iter_rgb));
            uint8_t r = (rgb_val >> 16) & 0xFF;
            uint8_t g = (rgb_val >> 8) & 0xFF;
            uint8_t b = rgb_val & 0xFF;

            // Octree aktualisieren und färben
            auto node = octree_->updateNode(octomap::point3d(pt_out.point.x, pt_out.point.y, pt_out.point.z), true);
            if (node) node->setColor(r, g, b);
            ++count;
        }

        RCLCPP_INFO(this->get_logger(), "Verarbeitete gültige Punkte: %zu", count);

        // Octomap-Nachricht publizieren
        octomap_msgs::msg::Octomap octomap_msg;
        octomap_msg.header = msg->header;
        if (octomap_msgs::fullMapToMsg(*octree_, octomap_msg)) {
            pub_->publish(octomap_msg);
            RCLCPP_INFO(this->get_logger(), "Octomap veröffentlicht (Frame: %s)", octomap_msg.header.frame_id.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Fehler beim Konvertieren der Octomap-Nachricht!");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::shared_ptr<octomap::ColorOcTree> octree_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
