This repository contains custom ROS2 packages developed for use with Gazebo simulations for navigation over a TCP/IP connection. The packages were tested with the ROS2 Humble distribution.

1. Clone the repository and build the workspace:

   git clone -- repo name --
   cd workspace_folder
   source /opt/ros/<distro>/setup.bash
   colcon build --symlink-install
   source install/setup.bash

3. To run the system, all three nodes must be launched in separate terminal windows using the following commands:
    ros2 run tcp_client_package tcp_client_node
    ros2 run location_package goal_executor.py
    ros2 run location_package coordinate_logger.py
   Or run launch file: 
    ros2 launch goals.launch.py

Important: Before building the workspace, make sure to update the IP address and port in the /tcp_client_package/tcp_client_package/tcp_client_node.py to match your system setup.
