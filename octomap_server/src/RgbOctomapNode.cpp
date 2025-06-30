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
        tf_listener_(tf_buffer_),
        frame_count_(0),
        prune_interval_(5),        // nur alle 5 Frames prunen
        publish_interval_(5),      // nur alle 5 Frames publizieren
        decimation_factor_(4)      // nur jeder 4. Punkt
    {
        RCLCPP_INFO(this->get_logger(), "Starte RgbOctomapNode...");
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth_registered/points", 10,
            std::bind(&RgbOctomapNode::pointCloudCallback, this, std::placeholders::_1));

        octree_ = std::make_shared<octomap::ColorOcTree>(0.03);  // Auflösung 5cm
        pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        ++frame_count_;

        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(
                "camera_color_optical_frame", msg->header.frame_id,
                tf2::TimePointZero, std::chrono::milliseconds(100));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF konnte nicht geholt werden: %s", ex.what());
            return;
        }

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_rgb(*msg, "rgb");

        // Berechne Sensor‐Ursprung in target_frame
        octomap::point3d sensor_origin(
          transform_stamped.transform.translation.x,
          transform_stamped.transform.translation.y,
          transform_stamped.transform.translation.z);

        size_t count = 0, idx = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
            if (++idx % decimation_factor_ != 0) continue;  // DECIMATION
            float x = *iter_x, y = *iter_y, z = *iter_z;

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

            // 1) Freiraum markieren: alle Zellen entlang des Strahls werden leer
            octomap::point3d endpoint(pt_out.point.x, pt_out.point.y, pt_out.point.z);
            octree_->insertRay(sensor_origin, endpoint, -1, true);

            // 2) Endpunkt als belegt und eingefärbt markieren
            if (auto node = octree_->updateNode(endpoint, true)) 
                node->setColor(r,g,b);
            ++count;
        }

        // PRUNE + LAZY_UPDATE nur alle prune_interval_ Frames
        if (frame_count_ % prune_interval_ == 0) {
            octree_->updateInnerOccupancy();
            octree_->prune();
        }

        // PUBLISH nur alle publish_interval_ Frames
        if (frame_count_ % publish_interval_ == 0) {
            octomap_msgs::msg::Octomap octomap_msg;
            octomap_msg.header = msg->header;
            if (octomap_msgs::fullMapToMsg(*octree_, octomap_msg)) {
                pub_->publish(octomap_msg);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::shared_ptr<octomap::ColorOcTree> octree_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    size_t frame_count_, prune_interval_, publish_interval_, decimation_factor_;
};
