#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import subprocess
import signal

class StressNode(Node):
    def __init__(self):
        super().__init__('stress_node')
        
        # Get the number of cores
        self.num_cores = os.cpu_count()
        self.get_logger().info(f"Detected {self.num_cores} CPU cores.")
        
        # Run stress with the number of cores
        self.stress_process = None
        self.run_stress()

    def run_stress(self):
        try:
            # Start stress command with the number of cores as the load argument
            self.get_logger().info("Starting stress command...")
            self.stress_process = subprocess.Popen(['stress', '--cpu', str(self.num_cores)])
            self.get_logger().info("Stress command launched successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to start stress: {e}")

    def destroy_node(self):
        # Terminate the stress process if it is still running
        if self.stress_process and self.stress_process.poll() is None:
            self.get_logger().info("Terminating stress process...")
            self.stress_process.send_signal(signal.SIGTERM)
            self.stress_process.wait()
            self.get_logger().info("Stress process terminated.")
        
        # Call the parent destroy_node() to complete the shutdown
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    stress_node = StressNode()
    
    try:
        rclpy.spin(stress_node)
    except KeyboardInterrupt:
        pass
    finally:
        stress_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
