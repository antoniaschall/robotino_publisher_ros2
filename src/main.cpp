#include "robotino_publisher/robotino_publisher.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<RobotinoPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Error: %s", e.what());
        std::cerr << "Error: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
