#include "RgbOctomapNode.cpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starte main()");
    rclcpp::spin(std::make_shared<RgbOctomapNode>());
    rclcpp::shutdown();
    return 0;
}