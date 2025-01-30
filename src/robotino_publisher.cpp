#include "robotino_publisher/robotino_publisher.hpp"
#include <stdexcept>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

using namespace rec::robotino::api2;

RobotinoPublisher::RobotinoPublisher()
: Node("robotino_publisher"),
  high_frequency_countdown_(0),
  default_interval_counter_(20),
  current_counter_(0),
  x_(0.0),
  y_(0.0),
  theta_(0.0),
  vx_(0.0f),
  vy_(0.0f),
  omega_(0.0f),
  sequence_(0),
  running_(true)
{
    // Declare parameter (IP), remove port references
    this->declare_parameter<std::string>("ip_address", "172.21.2.90");
    this->get_parameter("ip_address", ip_address_);

    RCLCPP_INFO(this->get_logger(),
                "Parameter loaded: ip=%s",
                ip_address_.c_str());

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("agv_position_data", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robotino_marker", 10);

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "detection_trigger", 10,
        std::bind(&RobotinoPublisher::triggerCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&RobotinoPublisher::publishData, this)
    );

    reconnect_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&RobotinoPublisher::attemptReconnection, this)
    );

    // Set Robotino IP
    setAddress(ip_address_.c_str());

    // Attempt initial connection
    try {
        connectToServer();
        RCLCPP_INFO(this->get_logger(), "Initial connection to Robotino at %s successful.", ip_address_.c_str());
        is_connected_ = true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Initial connection error: %s. Will keep trying...", e.what());
        is_connected_ = false;
    }

    // Start the event processing thread
    process_events_thread_ = std::thread(&RobotinoPublisher::processEventsLoop, this);
}

RobotinoPublisher::~RobotinoPublisher()
{
    running_ = false;
    if (process_events_thread_.joinable()) {
        process_events_thread_.join();
    }
    if (is_connected_) {
        disconnectFromServer();
        RCLCPP_INFO(this->get_logger(), "Disconnected from Robotino in destructor.");
    }
}

void RobotinoPublisher::publishData()
{
    if (!is_connected_) {
        RCLCPP_DEBUG(this->get_logger(), "Not connected; skipping publishData.");
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto time_since_last_data = now - last_data_time_.load();
    if (time_since_last_data > data_timeout_) {
        RCLCPP_WARN(this->get_logger(),
                    "No new data received for over %ld seconds. Marking as disconnected.",
                    std::chrono::duration_cast<std::chrono::seconds>(data_timeout_).count());
        is_connected_ = false;
        disconnectFromServer();
        return;
    }

    try {
        double x, y, theta;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            x = x_;
            y = y_;
            theta_ = theta;
            theta = theta_;
        }

        // Rotation angle (must be adapted)
        double theta_rot = 219.0 * M_PI / 180; 
        double cos_theta = std::cos(theta_rot);
        double sin_theta = std::sin(theta_rot);

        // Apply rotation
        double x_rotated = cos_theta * x - sin_theta * y;
        double y_rotated = sin_theta * x + cos_theta * y;

        // Apply offsets (must be adapted)
        double x_transformed = 0.0 - x_rotated;
        double y_transformed = 0.0 - y_rotated;

        // Adjust theta
        double theta_transformed = theta + theta_rot;

        // High-frequency logic
        if (high_frequency_countdown_ > 0) {
            --high_frequency_countdown_;
            if (high_frequency_countdown_ == 0) {
                RCLCPP_INFO(this->get_logger(), "High-frequency mode ended. Returning to default interval.");
            }
        } else {
            ++current_counter_;
            if (current_counter_ < default_interval_counter_) {
                return;
            }
            current_counter_ = 0;
        }

        // Create PoseStamped
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = this->get_name();

        msg.pose.position.x = x_transformed;
        msg.pose.position.y = y_transformed;
        msg.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_transformed);
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        RCLCPP_INFO(this->get_logger(),
                    "Publishing Transformed PoseStamped: (x=%.2f, y=%.2f, theta=%.2f)",
                    x_transformed, y_transformed, theta_transformed);
        publisher_->publish(msg);

        // Create and publish Marker
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = msg.header.stamp;
        marker.header.frame_id = msg.header.frame_id;
        marker.ns = "robotino";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = msg.pose;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_pub_->publish(marker);

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error in publishData: %s", e.what());
    }
}

void RobotinoPublisher::processEventsLoop()
{
    while (running_) {
        if (is_connected_) {
            try {
                processEvents();
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "processEvents failed: %s", e.what());
                is_connected_ = false;
                disconnectFromServer();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void RobotinoPublisher::readingsEvent(double x, double y, double phi,
                                      float vx, float vy, float omega,
                                      unsigned int sequence)
{
    last_data_time_.store(std::chrono::steady_clock::now());

    std::lock_guard<std::mutex> lock(data_mutex_);
    x_ = x;
    y_ = y;
    theta_ = phi;
    vx_ = vx;
    vy_ = vy;
    omega_ = omega;
    sequence_ = sequence;
}

void RobotinoPublisher::triggerCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Trigger received: '%s'", msg->data.c_str());
    if (msg->data == this->get_name()) {
        high_frequency_countdown_ = 20;
        current_counter_ = 0;
        RCLCPP_INFO(this->get_logger(), "High-frequency mode activated for 10 seconds.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Trigger does not match this node's name. Ignored.");
    }
}

void RobotinoPublisher::attemptReconnection()
{
    if (!is_connected_) {
        RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to Robotino...");
        try {
            connectToServer();
            RCLCPP_INFO(this->get_logger(), "Reconnection to Robotino at %s successful.",
                        ip_address_.c_str());
            is_connected_ = true;
            last_data_time_.store(std::chrono::steady_clock::now());
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Reconnection attempt failed: %s", e.what());
            is_connected_ = false;
        }
    }
}
