This repository contains custom ROS2 packages developed for use with Gazebo simulations for navigation over a TCP/IP connection. The packages were tested with the ROS2 Humble distribution.
Use any mobile app that can create a client which can then connect to the simulation.
Used application: TCP UDP Server & Client

## Getting started
Clone the repository and build the workspace:

```
   git clone https://github.com/albic98/goal_assignment.git
   cd workspace_folder
   source /opt/ros/<distro>/setup.bash
   colcon build --symlink-install
   source install/setup.bash
```

## Usage
To run the system, all three nodes must be launched in separate terminal windows using the following commands:

```
    ros2 run tcp_client_package tcp_client_node
    ros2 run location_package goal_executor
    ros2 run location_package coordinate_logger
```

   Or run launch file: 
```
    ros2 launch goals.launch.py
```

## Important 
Before building the workspace, make sure to update the IP address and port in the /tcp_client_package/tcp_client_package/tcp_client_node.py to match your system setup.
