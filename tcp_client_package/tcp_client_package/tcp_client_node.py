#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  
import socket
import time

class TCPBridgeNode(Node):
    def __init__(self):
        super().__init__('tcp_bridge_node')

        self.declare_parameter('listen_host', '192.168.42.134')
        self.declare_parameter('listen_port', 7020)
        self.declare_parameter('send_host', '10.182.42.133')
        self.declare_parameter('send_port', 7020)
        self.declare_parameter('confirmation_message', 'Message received!')
        self.declare_parameter('receive_timeout', 1.0)
        self.declare_parameter('send_timeout', 2.0)
        self.declare_parameter('topic_name', 'coordinates_topic')

        self.listen_host = self.get_parameter('listen_host').value
        self.listen_port = self.get_parameter('listen_port').value
        self.send_host = self.get_parameter('send_host').value
        self.send_port = self.get_parameter('send_port').value

        confirmation_message_value = self.get_parameter('confirmation_message').value
        if confirmation_message_value is None:
            confirmation_message_value = 'Message not received!'
        self.confirmation_msg = confirmation_message_value # confirmation_message_value.encode()

        self.receive_timeout = self.get_parameter('receive_timeout').value
        self.send_timeout = self.get_parameter('send_timeout').value

        topic_name = self.get_parameter('topic_name').value
        
        if not isinstance(topic_name, str) or topic_name is None:
            topic_name = 'coordinates_topic'
        self.publisher_ = self.create_publisher(String, topic_name, 10)

        self.process_status_subscription = self.create_subscription(String, 'process_status', self.process_status_callback, 10)
        
        self.listener_socket = None
        self.active_connection = None
        self.shutdown_requested = False
        self.setup_listener()

        self.timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info(f'Node started, listening on {self.listen_host}:{self.listen_port}')
        self.get_logger().info(f'Publishing messages to topic: {self.get_parameter("topic_name").value}')


    def process_status_callback(self, msg):
        """Callback za primanje poruka o uspjehu cilja"""
        # self.get_logger().info(f"Goal success message received: {msg.data}")
        self.send_message(f"{msg.data}")


    def setup_listener(self):
        """Postavi socket za slušanje"""
        try:
            if self.listener_socket:
                self.listener_socket.close()

            self.listener_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.listener_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.listener_socket.bind((self.listen_host, self.listen_port))
            self.listener_socket.listen()
            self.listener_socket.settimeout(self.receive_timeout)

        except Exception as e:
            self.get_logger().error(f'Error setting up listener: {str(e)}')
            time.sleep(1)
            if not self.shutdown_requested:
                self.setup_listener()


    def send_message(self, message: str):
        """Šalji potvrdnu poruku na definiranu adresu i port na mobilni uređaj"""
        try:
            if self.active_connection:
                # self.get_logger().info(f"Attempting to send: {message}")
                msg = (message + "\n").encode("utf-8")
                self.active_connection.sendall(msg)
                self.get_logger().info(f'Successfully sent message to client', throttle_duration_sec=1)
            else:
                self.get_logger().warn("No active connection to send message")
        except Exception as e:
            self.get_logger().error(f'Error sending message: {str(e)}')


    def publish_message(self, data):
        """Publisha primljenu poruku na topic"""
        try:
            msg = String()
            msg.data = data.decode() if isinstance(data, bytes) else str(data)
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Published to topic: {msg.data}', throttle_duration_sec=1)
        except Exception as e:
            self.get_logger().error(f'Error publishing message: {str(e)}')


    def main_loop(self):
        """Glavna obrada poruka"""
        if self.shutdown_requested:
            return
            
        try:
            if self.listener_socket is None:
                return
            if self.active_connection is None:
                # Wait for new client
                conn, addr = self.listener_socket.accept()
                self.get_logger().info(f"New connection from {addr[0]}:{addr[1]}")
                conn.settimeout(self.receive_timeout)
                self.active_connection = conn

            # Try receiving from active connection
            try:
                data = self.active_connection.recv(4096)
                if not data:
                    self.get_logger().warn("Client disconnected")
                    self.active_connection.close()
                    self.active_connection = None
                    return

                self.get_logger().info(f"Received: {data.decode(errors='ignore')}")
                # Send confirmation back
                self.send_message(self.confirmation_msg)
                # Publish to ROS
                self.publish_message(data)

            except socket.timeout:
                pass
            except Exception as e:
                self.get_logger().error(f'Connection error: {str(e)}')
                if self.active_connection:
                    self.active_connection.close()
                    self.active_connection = None

        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().error(f'Accept error: {str(e)}')
            if not self.shutdown_requested:
                time.sleep(1.0)
                self.setup_listener()


    def shutdown(self):
        """Čišćenje pri zatvaranju čvora"""
        self.shutdown_requested = True
        self.get_logger().info("Shutting down node...")
        
        if self.active_connection:
            try:
                self.active_connection.close()
            except:
                pass
                
        if self.listener_socket:
            try:
                self.listener_socket.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = TCPBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.shutdown()
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        node.get_logger().info("Node shutdown complete")

if __name__ == '__main__':
    main()
