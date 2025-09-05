#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import os
import time

class CoordinateLogger(Node):
    def __init__(self):
        super().__init__('coordinate_logger')
        self.subscription = self.create_subscription(String, 'coordinates_topic', self.listener_callback, 10)

        self.is_logging = False
        self.coordinates = []
        self.get_logger().info('Coordinate Logger Node started')

    def listener_callback(self, msg):
        data = msg.data.strip()


        divided_data = data.split('\n')

        print(f"Received: {divided_data}")
        
        
        for data in divided_data:
            data = data.strip()
            if data == "startFile":
                self.start_logging()
            elif data == "endFile":
                self.stop_logging()
            elif self.is_logging and data.startswith("(") and data.endswith(")"):
                self.process_coordinate_data(data)
            elif data == "home":
                self.get_logger().info("Received home command")
                self.process_coordinate_data("(197.4,280.835,-3.10)") # docking station
            else:
                self.get_logger().debug(f"Ignoring message: {data}")

    def start_logging(self):
        self.coordinates = []
        self.is_logging = True
        self.get_logger().info("Started logging coordinates")

    def stop_logging(self):
        if self.is_logging:
            self.is_logging = False
            self.save_to_yaml()
            self.get_logger().info(f"Stopped logging. Saved {len(self.coordinates)} coordinates")

    def process_coordinate_data(self, data):
        try:
            coords = list(map(float, data[1:-1].split(',')))
            print(coords)
            if len(coords) >= 3:
                coord_dict = {
                    'x': coords[0],
                    'y': coords[1],
                    # 'z': coords[2],
                    # 'roll': coords[3], # roll direction
                    # 'pitch': coords[4], # pitch direction
                    'yaw': coords[2]  # yaw direction
                }
                self.coordinates.append(coord_dict)
                self.get_logger().info(f"Logged coordinates: {coord_dict}")
            else:
                self.get_logger().warning(f"Invalid coordinate format: {data}")
        except Exception as e:
            self.get_logger().error(f"Error processing coordinates: {str(e)}")

    def save_to_yaml(self):
        timestamp = int(time.time())
        filename = f"coordinates_{timestamp}.yaml"
        
        try:
            with open(filename, 'w') as yaml_file:
                yaml.dump({'coordinates': self.coordinates}, yaml_file, default_flow_style=False)
            self.get_logger().info(f"Saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Error saving YAML: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
