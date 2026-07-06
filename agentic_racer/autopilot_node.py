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
        self.get_logger().info("ALGO 5: Ultra-Grip Equidistant Initialized!")
        
    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        num_ranges = len(ranges)
        front_idx = num_ranges // 2
        
        # --- ALGO 5: PD High-Speed Equidistant ---
        # To break the 11s lap time, we must raise the speed cap to 4.0 m/s.
        # To prevent tunneling at 4.0 m/s, we must eliminate understeer by using a 
        # Derivative (Kd) term to violently snap the steering into the apex early!
        
        # 2. Fixed 45-degree peripheral gaze
        # 45 degrees = index 45. We use a 20-degree cone (10 indices each side)
        left_cone = ranges[front_idx + 45 - 10 : front_idx + 45 + 10]
        right_cone = ranges[front_idx - 45 - 10 : front_idx - 45 + 10]
        front_cone = ranges[front_idx - 10 : front_idx + 10]
        
        # Filter infinities
        left_dist = min([min(r, 10.0) for r in left_cone])
        right_dist = min([min(r, 10.0) for r in right_cone])
        front_dist = min([min(r, 10.0) for r in front_cone])
        
        # 3. High-Gain Proportional Steering
        # The PD controller caused understeer because the Derivative term fights the 
        # Proportional term as the robot approaches the center line, which is fatal in 
        # a continuous curve! We revert to pure Proportional, but crank Kp to 2.5.
        error = left_dist - right_dist
        Kp = 2.5
        steering = error * Kp
        
        # Increased maximum steering limit to 4.0 rad/s for tighter turning radius at 3.0 m/s!
        steering = max(-4.0, min(4.0, steering))
        
        # 4. High-Speed Control
        # Retaining the max speed at 3.0 m/s per user request. 
        speed = 1.0 + (front_dist * 0.8) - (abs(steering) * 0.8)
        speed = max(1.5, min(3.0, speed))
        
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
