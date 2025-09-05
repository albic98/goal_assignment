
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    tcp_client_node = Node(
        package='tcp_client_package',
        executable='tcp_client_node',
        name='tcp_client_node',
        output='screen',
    )

    coordinate_logger_node = Node(
        package='location_package',
        executable='coordinate_logger',
        name='coordinate_logger',
        output='screen'
    )

    goal_executor_node = Node(
        package='location_package',
        executable='goal_executor',
        name='goal_executor',
        output='screen'
    )

    return LaunchDescription([
        tcp_client_node,
        coordinate_logger_node,
        goal_executor_node,
    ])