#ifndef ROBOTINO_PUBLISHER_HPP
#define ROBOTINO_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rec/robotino/api2/Com.h>
#include <rec/robotino/api2/Odometry.h>

#include <chrono>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <ctime>
#include <string>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>

class RobotinoPublisher 
    : public rclcpp::Node
    , public rec::robotino::api2::Com
    , public rec::robotino::api2::Odometry
{
public:
    RobotinoPublisher();
    ~RobotinoPublisher();

private:
    void publishData();
    void processEventsLoop();
    void readingsEvent(double x, double y, double phi, float vx, float vy, float omega, unsigned int sequence) override;
    void triggerCallback(const std_msgs::msg::String::SharedPtr msg);
    void attemptReconnection();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;

    int high_frequency_countdown_;
    int default_interval_counter_;
    int current_counter_;

    // IP
    std::string ip_address_;

    // Robotino odometry values
    double x_;
    double y_;
    double theta_;
    float vx_;
    float vy_;
    float omega_;
    unsigned int sequence_;

    std::mutex data_mutex_;
    std::thread process_events_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> is_connected_{false};
    std::atomic<std::chrono::steady_clock::time_point> last_data_time_{std::chrono::steady_clock::now()};

    const std::chrono::seconds data_timeout_{5};
};

#endif // ROBOTINO_PUBLISHER_HPP
