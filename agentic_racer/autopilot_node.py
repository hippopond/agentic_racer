import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AutopilotNode(Node):
    def __init__(self):
        super().__init__('autopilot_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        self.publisher = self.create_publisher(Twist, '/cmd_vel_test', 10)
        self.get_logger().info("Equidistant LiDAR Autopilot Initialized.")
        
    def scan_callback(self, msg):
        ranges = msg.ranges
        num_ranges = len(ranges)
        front_idx = num_ranges // 2
        
        # 1. Read Distances
        front_dist = min(ranges[front_idx - 10 : front_idx + 10])
        left_dist = min(ranges[front_idx + 30 : front_idx + 90])
        right_dist = min(ranges[front_idx - 90 : front_idx - 30])
        
        # 2. Equidistant Steering Logic
        # If left is further than right, we are too close to the right wall -> Turn Left (Positive)
        error = left_dist - right_dist
        
        # Proportional controller for steering
        Kp = 1.5
        steering = error * Kp
        
        # Clamp steering to prevent spinning out
        steering = max(-2.5, min(2.5, steering))
        
        # 3. Adaptive Speed Control
        # Slow down if we are steering hard (cornering)
        speed = 1.0 - abs(steering) * 0.4
        speed = max(0.5, min(1.0, speed))
        
        # Emergency safety brake
        if front_dist < 0.5:
            speed = 0.0
            
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = steering
        
        self.publisher.publish(twist)
        
        # Debug Logger every ~1 second (assuming 10Hz scan)
        if not hasattr(self, 'log_counter'): self.log_counter = 0
        self.log_counter += 1
        if self.log_counter % 10 == 0:
            self.get_logger().info(f"Speed: {speed:.2f} | L: {left_dist:.2f} R: {right_dist:.2f} | Steer: {steering:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = AutopilotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
