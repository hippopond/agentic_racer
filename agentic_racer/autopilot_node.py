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
        self.get_logger().info("ALGO 4: High-Grip Equidistant Initialized!")
        
    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        num_ranges = len(ranges)
        front_idx = num_ranges // 2
        
        # --- ALGO 4: High-Grip Equidistant ---
        # Algo 3 was actually tunneling through the walls because it was driving too fast 
        # and understeering into the jagged collision meshes of the new curved track.
        
        # 2. Fixed 45-degree peripheral gaze
        # 45 degrees = index 45. We use a 20-degree cone (10 indices each side)
        left_cone = ranges[front_idx + 45 - 10 : front_idx + 45 + 10]
        right_cone = ranges[front_idx - 45 - 10 : front_idx - 45 + 10]
        front_cone = ranges[front_idx - 10 : front_idx + 10]
        
        # Filter infinities
        left_dist = min([min(r, 10.0) for r in left_cone])
        right_dist = min([min(r, 10.0) for r in right_cone])
        front_dist = min([min(r, 10.0) for r in front_cone])
        
        # 3. Proportional Steering
        # Kp raised to 2.0 to forcefully track the center line and prevent understeer!
        error = left_dist - right_dist
        Kp = 2.0
        steering = error * Kp
        
        # Extremely wide steering limits for 5.0m/s emergency maneuvers
        steering = max(-3.0, min(3.0, steering))
        
        # 4. Anti-Tunneling & Anti-Pirouette Speed Control
        # We MUST brake when steering to prevent ramming the walls, but if speed drops too low 
        # (e.g. 0.5 m/s) while steering is high (2.8 rad/s), the robot enters a pirouette loop 
        # (spinning in place) and visually looks "stopped". 
        # We guarantee a minimum speed of 1.5 m/s to maintain forward momentum!
        speed = 1.0 + (front_dist * 0.6) - (abs(steering) * 0.8)
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
