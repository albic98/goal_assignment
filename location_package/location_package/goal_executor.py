#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
# from location_interfaces.srv import ExecuteMission
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import yaml
import math
import os

class GoalExecutor(Node):
    def __init__(self):
        super().__init__('goal_executor')

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self.current_file = None
        self.coordinates = []
        self.current_index = 0
        self.mission_active = False
        self.goal_reached = False

        self.goal_location_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.process_status_publisher = self.create_publisher(String, 'process_status', 10)

        self.create_timer(1.0, self.check_for_mission)
        self.get_logger().info("Goal Executor Node started - waiting for missions")

    def check_for_mission(self):
        if self.mission_active:
            return

        for file in os.listdir('.'):
            if file.endswith('.yaml') and file.startswith('coordinates'):
                self.start_mission(file)
                return

    def start_mission(self, filename):
        try:
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)
                if not data or 'coordinates' not in data:
                    self.get_logger().error("Invalid YAML format")
                    return
                
                self.coordinates = data['coordinates']
                self.current_index = 0
                self.current_file = filename
                self.mission_active = True
                self.get_logger().info(f"Starting mission with {len(self.coordinates)} goals from {filename}")
                self.process_status_publisher.publish(String(data=f"Starting mission with {len(self.coordinates)} goals"))
                self.execute_next_goal()

        except Exception as e:
            self.get_logger().error(f"Error loading mission file: {str(e)}")

    def execute_next_goal(self):
        if self.current_index >= len(self.coordinates):
            self.get_logger().info("Mission complete!")
            self.cleanup_mission()
            return
        
        self.goal_reached = False
        coord = self.coordinates[self.current_index]
        self.current_index += 1
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = coord['x']
        goal_msg.pose.pose.position.y = coord['y']
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz = 0.0, 0.0 , coord['yaw']
        qw = math.sqrt(max(0.0, 1.0 - qx**2 - qy**2 - qz**2))

        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"Executing goal {self.current_index}/{len(self.coordinates)} to ({coord['x']}, {coord['y']})")
        self.process_status_publisher.publish(String(data=f"Executing {self.current_index}. of {len(self.coordinates)} goals!"))
        if not self.goal_reached:
            self.goal_location_publisher.publish(goal_msg.pose)
            self.goal_reached = True

        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.execute_next_goal()
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted, waiting for result...')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal succeeded!')
            self.process_status_publisher.publish(String(data=f"Robot has reached {self.current_index}. of {len(self.coordinates)} goals!"))
            self.goal_reached = True
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
            self.process_status_publisher.publish(String(data=f"Failed to reach {self.current_index}. of {len(self.coordinates)} goals!"))

        self.execute_next_goal()

    def cleanup_mission(self):
        try:
            if self.current_file is not None:
                os.remove(self.current_file)
                self.get_logger().info(f"Removed mission file: {self.current_file}")
        except Exception as e:
            self.get_logger().error(f"Error removing mission file: {str(e)}")

        self.current_file = None
        self.coordinates = []
        self.current_index = 0
        self.mission_active = False

def main(args=None):
    rclpy.init(args=args)
    executor = GoalExecutor()
    multi_exec = MultiThreadedExecutor()
    multi_exec.add_node(executor)
    
    try:
        multi_exec.spin()
    except KeyboardInterrupt:
        executor.get_logger().info("Shutting down")
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
