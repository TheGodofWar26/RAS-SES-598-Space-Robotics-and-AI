#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np
import matplotlib.pyplot as plt
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from collections import deque
import threading

class EarthquakeForceGenerator(Node):
    def __init__(self):
        super().__init__('earthquake_force_generator')
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher for the cart force
        self.force_publisher = self.create_publisher(
            Float64, 
            '/model/cart_pole/joint/cart_to_base/cmd_force',
            qos_profile
        )
        
        # Publisher for force visualization
        self.viz_publisher = self.create_publisher(
            Float64,
            '/earthquake_force',
            qos_profile
        )
        
        # Parameters for earthquake simulation
        self.declare_parameter('base_amplitude', 100.0)  # Base force amplitude in N
        self.declare_parameter('frequency_range', [0.5, 4.0])  # Frequency range in Hz
        self.declare_parameter('update_rate', 50.0)  # Update rate in Hz
        
        # Timer for force updates
        update_period = 1.0 / self.get_parameter('update_rate').value
        self.timer = self.create_timer(update_period, self.generate_force)
        
        # Time tracking for continuous wave generation
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Generate random frequencies for the earthquake waves
        freq_range = self.get_parameter('frequency_range').value
        self.frequencies = np.random.uniform(freq_range[0], freq_range[1], 5)
        self.phase_shifts = np.random.uniform(0, 2*np.pi, 5)
        
        # Data storage for plotting
        self.time_data = deque(maxlen=500)
        self.force_data = deque(maxlen=500)
        
        self.get_logger().info('Earthquake Force Generator started')

        # Start the plotting thread
        self.plot_thread = threading.Thread(target=self.plot_force, daemon=True)
        self.plot_thread.start()

    def generate_force(self):
        """Generate earthquake-like force using superposition of sine waves"""
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        base_amplitude = self.get_parameter('base_amplitude').value
        
        # Superimpose multiple sine waves with different frequencies
        force = 0.0
        for freq, phase in zip(self.frequencies, self.phase_shifts):
            # Add randomized amplitude variation
            amplitude = base_amplitude * np.random.uniform(0.8, 1.2)
            force += amplitude * np.sin(2 * np.pi * freq * current_time + phase)
        
        # Add some random noise
        force += np.random.normal(0, base_amplitude * 0.1)
        
        # Store data for plotting
        self.time_data.append(current_time)
        self.force_data.append(force)
        
        # Create and publish the force messages
        msg = Float64()
        msg.data = float(force)
        self.force_publisher.publish(msg)
        self.viz_publisher.publish(msg)  # Publish the same force for visualization
    
    def plot_force(self):
        """Real-time plot of the earthquake force"""
        plt.ion()
        fig, ax = plt.subplots()
        ax.set_title("Earthquake Force Over Time")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Force (N)")
        line, = ax.plot([], [], 'b-')
        
        while rclpy.ok():
            if self.time_data:
                ax.set_xlim(max(0, self.time_data[0] - 1), self.time_data[-1] + 1)
                line.set_data(self.time_data, self.force_data)
                ax.relim()
                ax.autoscale_view()
                plt.pause(0.05)
        
        plt.ioff()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = EarthquakeForceGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
