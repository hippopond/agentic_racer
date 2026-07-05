import rclpy
from rclpy.node import Node
import json
import os

class RefereeNode(Node):
    def __init__(self):
        super().__init__('referee_node')
        self.get_logger().info("Referee Node initialized! Waiting for the race to start...")
        
        # State Tracking
        self.laps_completed = 0
        self.crashes = 0
        self.start_time = self.get_clock().now()
        
        # In a real F1TENTH eval, we would subscribe to /odom and /scan here
        # self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def generate_report(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        report = {
            "laps_completed": self.laps_completed,
            "crashes": self.crashes,
            "elapsed_time_sec": elapsed_time
        }
        
        # Save report for the LLM Agent to read
        report_path = os.path.join(os.getcwd(), 'test_report.json')
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=4)
        self.get_logger().info(f"Race finished. Evaluation report saved to {report_path}")

def main(args=None):
    rclpy.init(args=args)
    node = RefereeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.generate_report()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
