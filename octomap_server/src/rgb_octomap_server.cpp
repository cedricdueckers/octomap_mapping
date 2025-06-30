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
#include "octomap_msgs/srv/get_octomap.hpp"

class RgbOctomapNode : public rclcpp::Node {
public:
    RgbOctomapNode() : Node("rgb_octomap_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_),
        frame_count_(0),
        prune_interval_(5),        // Prune the octree every 5 frames
        publish_interval_(5),      // Publish the octree every 5 frames
        decimation_factor_(4)      // Process every 4th point in the point cloud
    {
        // Declare and get parameters
        this->declare_parameter<double>("resolution", 0.03);
        this->declare_parameter<std::string>("camera_frame", "base_link");
        this->declare_parameter<std::string>("world_frame", "odom");
        this->declare_parameter<std::string>("pointcloud_topic", "/camera/depth_registered/points");

        double resolution = this->get_parameter("resolution").as_double();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        world_frame_ = this->get_parameter("world_frame").as_string();
        pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "Starting rgb_octomap_node with resolution: %.2f, camera_frame: %s, world_frame: %s, pointcloud_topic: %s",
                    resolution, camera_frame_.c_str(), world_frame_.c_str(), pointcloud_topic_.c_str());

        // Subscribe to the point cloud topic
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, 10, std::bind(&RgbOctomapNode::pointCloudCallback, this, std::placeholders::_1));

        // Initialize the octree with the specified resolution
        octree_ = std::make_shared<octomap::ColorOcTree>(resolution);

        // Publisher for the color octomap
        pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("color_octomap", 10);

        // add this:
        srv_ = this->create_service<octomap_msgs::srv::GetOctomap>(
          "color_octomap",
          std::bind(&RgbOctomapNode::onGetColorOctomap, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        ++frame_count_;

        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            // Retrieve the transformation from the world frame to the camera frame
            transform_stamped = tf_buffer_.lookupTransform(
                world_frame_,  // Use the parameter for the world frame
                camera_frame_, // Use the parameter for the camera frame
                tf2::TimePointZero, std::chrono::milliseconds(100));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF not connected: %s", ex.what());
            return;
        }

        // Transform the sensor origin to the "odom" frame
        octomap::point3d sensor_origin(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z);

        // Iterate through the point cloud data
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_rgb(*msg, "rgb");

        size_t count = 0, idx = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
            if (++idx % decimation_factor_ != 0) continue;  // Skip points based on decimation factor
            float x = *iter_x, y = *iter_y, z = *iter_z;

            // Filter out invalid points (NaN) and points outside the depth range
            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            if (z < 0.3 || z > 5.0) continue;  // Only process points within the valid range

            // Transform the point to the "odom" frame
            geometry_msgs::msg::PointStamped pt_in, pt_out;
            pt_in.header = msg->header;
            pt_in.point.x = x;
            pt_in.point.y = y;
            pt_in.point.z = z;
            tf2::doTransform(pt_in, pt_out, transform_stamped);

            // Extract RGB values from the float32 format (standard PCL format)
            uint32_t rgb_val = *reinterpret_cast<const uint32_t*>(&(*iter_rgb));
            uint8_t r = (rgb_val >> 16) & 0xFF;
            uint8_t g = (rgb_val >> 8) & 0xFF;
            uint8_t b = rgb_val & 0xFF;

            // 1) Mark free space: all cells along the ray are marked as free
            octomap::point3d endpoint(pt_out.point.x, pt_out.point.y, pt_out.point.z);
            octree_->insertRay(sensor_origin, endpoint, -1, true);

            // 2) Mark the endpoint as occupied and set its color
            if (auto node = octree_->updateNode(endpoint, true)) 
                node->setColor(r, g, b);
            ++count;
        }

        // PRUNE + LAZY_UPDATE: Perform pruning every prune_interval_ frames
        if (frame_count_ % prune_interval_ == 0) {
            octree_->updateInnerOccupancy();
            octree_->prune();
        }

        // PUBLISH: Publish the octomap every publish_interval_ frames
        if (frame_count_ % publish_interval_ == 0) {
            octomap_msgs::msg::Octomap octomap_msg;
            octomap_msg.header.frame_id = "odom"; // Set the frame to "odom"
            octomap_msg.header.stamp = msg->header.stamp;
            if (octomap_msgs::fullMapToMsg(*octree_, octomap_msg)) {
                pub_->publish(octomap_msg);
            }
        }
    }

    // new callback:
    void onGetColorOctomap(
      const std::shared_ptr<octomap_msgs::srv::GetOctomap::Request> /*req*/,
      std::shared_ptr<octomap_msgs::srv::GetOctomap::Response> res)
    {
      res->map.header.frame_id = world_frame_;
      res->map.header.stamp = now();
      if (!octomap_msgs::fullMapToMsg(*octree_, res->map)) {
        RCLCPP_ERROR(get_logger(), "Failed to serialize color octomap");
      }
    }

    // ROS 2 subscription for the point cloud
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    
    // Octree for storing the 3D map with color information
    std::shared_ptr<octomap::ColorOcTree> octree_;
    
    // ROS 2 publisher for the octomap
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_;
    
    // TF2 buffer and listener for handling transformations
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Frame counters and configuration parameters
    size_t frame_count_, prune_interval_, publish_interval_, decimation_factor_;
    std::string camera_frame_, world_frame_, pointcloud_topic_;

    rclcpp::Service<octomap_msgs::srv::GetOctomap>::SharedPtr srv_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting RgbOctomapNode...");
    rclcpp::spin(std::make_shared<RgbOctomapNode>());
    rclcpp::shutdown();
    return 0;
}