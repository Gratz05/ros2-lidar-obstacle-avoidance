# ROS2 LIDAR Obstacle Avoidance using Gazebo

Autonomous obstacle avoidance using LIDAR data in ROS 2 with TurtleBot3 in Gazebo simulation. The robot subscribes to `/scan` and publishes velocity to `/cmd_vel` to avoid collisions in real time using Python.

## Tech Used
- ROS 2 Humble
- Python (rclpy)
- Gazebo
- TurtleBot3
- LaserScan, Twist topics

## How to Run

Terminal-1

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


Terminal-2

```bash
colcon build
source install/setup.bash
ros2 run lidar_obstacle_avoidance lidar_node
