This project provides a ROS 2 node that communicates with a Robotino platform via the rec_robotino_api2 library. It retrieves odometry data and publishes it as both a PoseStamped message and a Marker for RViz visualization. The node can adjust its publish frequency in response to a trigger message, and it includes automatic reconnection logic if the connection to Robotino is lost.

MAIN FEATURES:

- Retrieves Robotino odometry data (x, y, theta).
- Publishes a geometry_msgs/PoseStamped message (topic: "agv_position_data").
- Publishes a visualization_msgs/Marker (topic: "robotino_marker") for RViz.
- Listens for a trigger topic ("detection_trigger") to temporarily boost publish frequency.
- Attempts automatic reconnection every 10 seconds if no data is received for 5 seconds.

OVERVIEW:

1. CMakeLists.txt and package.xml: Define ROS 2 package dependencies and build instructions.
2. launch/robotino_publisher_launch.py: Example launch file that starts the Robotino Publisher node with an IP address parameter.
3. include/robotino_publisher/robotino_publisher.hpp: Declares the RobotinoPublisher class (inheriting from rclcpp::Node, rec::robotino::api2::Com, and rec::robotino::api2::Odometry).
4. src/robotino_publisher.cpp: Implements the core logic for connecting to Robotino, publishing data, handling triggers, and automatically attempting reconnection.
5. src/main.cpp: Entry point. Creates and spins the RobotinoPublisher node within ROS 2.

DEPENDENCIES:

- ROS 2 (rclcpp, std_msgs, geometry_msgs, visualization_msgs, tf2)
- rec_robotino_api2 libraries and headers (expected in /opt/robotino/lib and /opt/robotino/include)

INSTALLATION AND BUILD:

1. Create a ROS 2 workspace if you have not already. 
2. Clone this repository into the src folder
3. Build with colcon
4. Source the setup file 

USAGE AND MODIFICATIONS:

- To run directly: ros2 run robotino_publisher robotino_publisher
- To run via the launch file: ros2 launch robotino_publisher robotino_publisher_launch.py
- You can modify the "ip_address" parameter in the launch file or via command line: ros2 run robotino_publisher robotino_publisher --ros-args -p ip_address:=192.168.0.100
- You must also modify the rotation angle and offsets for the odometry data in RViz if the robotino's coordinate system or origin is configured skewed like mine was.

TRIGGER BEHAVIOR:

- The node subscribes to the "detection_trigger" topic (std_msgs/String).
- If the incoming message matches the node's name ("robotino_publisher"), it activates a higher publish rate for 10 seconds.

AUTOMATIC RECONNECTION:

If the node does not receive fresh odometry data for 5 seconds, it marks the connection as lost, disconnects, and attempts to reconnect every 10 seconds.
