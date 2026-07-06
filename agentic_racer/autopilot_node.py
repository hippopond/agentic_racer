import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class AutopilotNode(Node):
    def __init__(self):
        super().__init__('autopilot_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        self.publisher = self.create_publisher(Twist, '/cmd_vel_test', 10)
        self.get_logger().info("Predictive Equidistant (Algo 3) Initialized!")
        
    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        num_ranges = len(ranges)
        front_idx = num_ranges // 2
        
        # 1. Look Ahead Diagonally (Predictive)
        # We extract two 20-degree cones pointing exactly 45 degrees left and right.
        # This allows the robot to "see" the curve long before it enters the apex!
        left_cone = ranges[front_idx + 35 : front_idx + 55]
        right_cone = ranges[front_idx - 55 : front_idx - 35]
        front_cone = ranges[front_idx - 10 : front_idx + 10]
        
        # Filter infinities
        left_dist = min([min(r, 10.0) for r in left_cone])
        right_dist = min([min(r, 10.0) for r in right_cone])
        front_dist = min([min(r, 10.0) for r in front_cone])
        
        # 2. Predictive Equidistant Steering
        # By balancing the diagonal distances, the robot naturally steers into the curve
        # smoothly, avoiding the twitchiness of standard wall-followers.
        error = left_dist - right_dist
        
        Kp = 0.8
        steering = error * Kp
        
        # Unclamp the steering so it can actually make sharp hairpin turns!
        steering = max(-2.5, min(2.5, steering))
        
        # 3. Predictive Speed Control
        # If the track is clear straight ahead, and we aren't turning hard, punch it.
        if front_dist > 3.0 and abs(steering) < 0.3:
            speed = 3.0
        else:
            # Smoothly brake based on how close the front wall is
            speed = 0.5 + (front_dist * 0.4) - (abs(steering) * 0.5)
            
        speed = max(0.5, min(3.0, speed))
        
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = steering
        self.publisher.publish(twist)
        
        # Debug Logger every ~1 second
        if not hasattr(self, 'log_counter'): self.log_counter = 0
        self.log_counter += 1
        if self.log_counter % 10 == 0:
            self.get_logger().info(f"Speed: {speed:.2f} | L: {left_dist:.1f} R: {right_dist:.1f} | Steer: {steering:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = AutopilotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
