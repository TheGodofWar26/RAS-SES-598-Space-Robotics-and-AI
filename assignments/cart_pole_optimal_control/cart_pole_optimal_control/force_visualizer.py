# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point
# from sensor_msgs.msg import JointState
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# import numpy as np
# import matplotlib.pyplot as plt

# class ForceVisualizer(Node):
#     def __init__(self):
#         super().__init__('force_visualizer')
        
#         # Create QoS profiles
#         sensor_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )
        
#         # Subscribe to control force command
#         self.force_sub = self.create_subscription(
#             Float64,
#             '/model/cart_pole/joint/cart_to_base/cmd_force',
#             self.control_force_callback,
#             10)
            
#         # Subscribe to earthquake force
#         self.earthquake_sub = self.create_subscription(
#             Float64,
#             '/earthquake_force',  # We'll need to publish this from earthquake generator
#             self.earthquake_force_callback,
#             10)
            
#         # Subscribe to joint states using the republished topic
#         self.joint_state_sub = self.create_subscription(
#             JointState,
#             'joint_states',  # Standard ROS2 joint states topic
#             self.joint_state_callback,
#             sensor_qos)
            
#         # Publishers for force markers
#         self.control_marker_pub = self.create_publisher(Marker, '/control_force_marker', 10)
#         self.earthquake_marker_pub = self.create_publisher(Marker, '/earthquake_force_marker', 10)
        
#         self.cart_position = 0.0
#         self.control_force = 0.0
#         self.earthquake_force = 0.0
#         self.get_logger().info('Force Visualizer node started')
        
#     def joint_state_callback(self, msg):
#         # Get cart position from joint states
#         try:
#             cart_idx = msg.name.index('cart_to_base')
#             self.cart_position = msg.position[cart_idx]
#             # Update both force visualizations when position changes
#             self.publish_control_force()
#             self.publish_earthquake_force()
#         except ValueError:
#             self.get_logger().warn('cart_to_base joint not found in joint states')
            
#     def control_force_callback(self, msg):
#         self.control_force = msg.data
#         self.publish_control_force()
        
#     def earthquake_force_callback(self, msg):
#         self.earthquake_force = msg.data
#         self.publish_earthquake_force()
        
#     def create_force_marker(self, force, z_offset, is_control=True):
#         marker = Marker()
#         marker.header.frame_id = "world"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "control_force" if is_control else "earthquake_force"
#         marker.id = 0
#         marker.type = Marker.ARROW
#         marker.action = Marker.ADD
        
#         # Set the start point at the cart's position
#         start = Point()
#         start.x = self.cart_position
#         start.y = 0.0
#         start.z = 0.15 + z_offset  # Offset vertically to show different forces
        
#         # Set end point based on force magnitude (scaled for visualization)
#         end = Point()
#         scale = 0.1  # Scale factor to make force visible
#         end.x = self.cart_position + force * scale
#         end.y = 0.0
#         end.z = 0.15 + z_offset
        
#         marker.points = [start, end]
        
#         # Set the arrow properties
#         marker.scale.x = 0.02  # shaft diameter
#         marker.scale.y = 0.04  # head diameter
#         marker.scale.z = 0.1   # head length
        
#         # Color based on force type and direction
#         marker.color.a = 1.0
#         if is_control:
#             # Control force: red for positive, blue for negative
#             if force > 0:
#                 marker.color.r = 1.0
#                 marker.color.g = 0.0
#                 marker.color.b = 0.0
#             else:
#                 marker.color.r = 0.0
#                 marker.color.g = 0.0
#                 marker.color.b = 1.0
#         else:
#             # Earthquake force: orange for positive, purple for negative
#             if force > 0:
#                 marker.color.r = 1.0
#                 marker.color.g = 0.65
#                 marker.color.b = 0.0
#             else:
#                 marker.color.r = 0.5
#                 marker.color.g = 0.0
#                 marker.color.b = 0.5
                
#         return marker
            
#     def publish_control_force(self):
#         marker = self.create_force_marker(self.control_force, 0.0, True)
#         self.control_marker_pub.publish(marker)
        
#     def publish_earthquake_force(self):
#         marker = self.create_force_marker(self.earthquake_force, 0.1, False)
#         self.earthquake_marker_pub.publish(marker)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ForceVisualizer()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
import threading
import time

class ForceVisualizer(Node):
    def __init__(self):
        super().__init__('force_visualizer')
        
        # Create QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to control force command
        self.force_sub = self.create_subscription(
            Float64,
            '/model/cart_pole/joint/cart_to_base/cmd_force',
            self.control_force_callback,
            10)
            
        # Subscribe to earthquake force
        self.earthquake_sub = self.create_subscription(
            Float64,
            '/earthquake_force',  # We'll need to publish this from earthquake generator
            self.earthquake_force_callback,
            10)
            
        # Subscribe to joint states using the republished topic
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',  # Standard ROS2 joint states topic
            self.joint_state_callback,
            sensor_qos)
            
        # Publishers for force markers
        self.control_marker_pub = self.create_publisher(Marker, '/control_force_marker', 10)
        self.earthquake_marker_pub = self.create_publisher(Marker, '/earthquake_force_marker', 10)
        
        self.cart_position = 0.0
        self.control_force = 0.0
        self.earthquake_force = 0.0
        self.time_start = time.time()
        
        # Data logging for plotting
        self.time_log = []
        self.cart_position_log = []
        self.control_force_log = []
        self.earthquake_force_log = []

        # Start the plotting thread
        self.plot_thread = threading.Thread(target=self.plot_data)
        self.plot_thread.daemon = True
        self.plot_thread.start()

        self.get_logger().info('Force Visualizer node started')
        
    def joint_state_callback(self, msg):
        try:
            cart_idx = msg.name.index('cart_to_base')
            self.cart_position = msg.position[cart_idx]
            self.log_data()
            self.publish_control_force()
            self.publish_earthquake_force()
        except ValueError:
            self.get_logger().warn('cart_to_base joint not found in joint states')
            
    def control_force_callback(self, msg):
        self.control_force = msg.data
        self.log_data()
        self.publish_control_force()
        
    def earthquake_force_callback(self, msg):
        self.earthquake_force = msg.data
        self.log_data()
        self.publish_earthquake_force()
        
    def log_data(self):
        """ Logs the time, cart position, and forces for plotting. """
        current_time = time.time() - self.time_start
        self.time_log.append(current_time)
        self.cart_position_log.append(self.cart_position)
        self.control_force_log.append(self.control_force)
        self.earthquake_force_log.append(self.earthquake_force)

    def plot_data(self):
        """ Continuously updates the plots in real-time. """
        plt.ion()  # Enable interactive mode
        fig, axs = plt.subplots(3, 1, figsize=(8, 8))

        while rclpy.ok():
            if len(self.time_log) > 1:
                axs[0].cla()
                axs[1].cla()
                axs[2].cla()
                
                # Plot Cart Position
                axs[0].plot(self.time_log, self.cart_position_log, 'b-', label='Cart Position')
                axs[0].set_ylabel('Position (m)')
                axs[0].set_title('Cart Position over Time')
                axs[0].legend()
                axs[0].grid(True)

                # Plot Control Force
                axs[1].plot(self.time_log, self.control_force_log, 'r-', label='Control Force')
                axs[1].set_ylabel('Force (N)')
                axs[1].set_title('Control Force over Time')
                axs[1].legend()
                axs[1].grid(True)

                # Plot Earthquake Force
                axs[2].plot(self.time_log, self.earthquake_force_log, 'g-', label='Earthquake Force')
                axs[2].set_xlabel('Time (s)')
                axs[2].set_ylabel('Force (N)')
                axs[2].set_title('Earthquake Force over Time')
                axs[2].legend()
                axs[2].grid(True)

                plt.draw()
                plt.pause(0.1)

        plt.ioff()
        plt.show()

    def create_force_marker(self, force, z_offset, is_control=True):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "control_force" if is_control else "earthquake_force"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        start = Point()
        start.x = self.cart_position
        start.y = 0.0
        start.z = 0.15 + z_offset  
        
        end = Point()
        scale = 0.1  
        end.x = self.cart_position + force * scale
        end.y = 0.0
        end.z = 0.15 + z_offset
        
        marker.points = [start, end]
        
        marker.scale.x = 0.02  
        marker.scale.y = 0.04  
        marker.scale.z = 0.1   
        
        marker.color.a = 1.0
        if is_control:
            if force > 0:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
        else:
            if force > 0:
                marker.color.r = 1.0
                marker.color.g = 0.65
                marker.color.b = 0.0
            else:
                marker.color.r = 0.5
                marker.color.g = 0.0
                marker.color.b = 0.5
                
        return marker
            
    def publish_control_force(self):
        marker = self.create_force_marker(self.control_force, 0.0, True)
        self.control_marker_pub.publish(marker)
        
    def publish_earthquake_force(self):
        marker = self.create_force_marker(self.earthquake_force, 0.1, False)
        self.earthquake_marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ForceVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
