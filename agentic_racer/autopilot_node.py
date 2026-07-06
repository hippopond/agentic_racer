import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class AutopilotNode(Node):
    def __init__(self):
        super().__init__('autopilot_node')
        
        self.max_forward_speed = 4.0
        self.steering_gain = 2.0
        self.lookahead_distance = 1.0
        
        # PID parameters
        self.kp = 1.2
        self.kd = 2.0
        self.prev_error = 0.0
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        # Important: maintain the publisher on /cmd_vel_test as requested
        self.publisher = self.create_publisher(Twist, '/cmd_vel_test', 10)
        
    def scan_callback(self, msg):
        ranges = msg.ranges
        num_ranges = len(ranges)
        
        if num_ranges == 0:
            return
            
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def scan_callback(self, msg):
        ranges = msg.ranges
        num_ranges = len(ranges)
        
        if num_ranges == 0:
            return
            
        if self.current_yaw is not None:
            cross_product = self.current_x * math.sin(self.current_yaw) - self.current_y * math.cos(self.current_yaw)
            if cross_product < -0.5:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 2.5
                self.get_logger().warn("Wrong way detected! Executing U-turn.")
                self.publisher.publish(twist)
                return
            
        front_idx = num_ranges // 2
        idx_per_deg = num_ranges / 360.0
        
        # Slices for left, right, and front based on degrees
        r_start = int(front_idx - 70 * idx_per_deg)
        r_end = int(front_idx - 10 * idx_per_deg)
        
        l_start = int(front_idx + 10 * idx_per_deg)
        l_end = int(front_idx + 70 * idx_per_deg)
        
        f_start = int(front_idx - 20 * idx_per_deg)
        f_end = int(front_idx + 20 * idx_per_deg)
        
        right_cone = [r for r in ranges[r_start:r_end] if 0.1 < r < 10.0]
        left_cone = [r for r in ranges[l_start:l_end] if 0.1 < r < 10.0]
        front_cone = [r for r in ranges[f_start:f_end] if 0.1 < r < 10.0]
        
        # Average distance to walls for center-line following
        avg_right = sum(right_cone) / len(right_cone) if right_cone else 3.0
        avg_left = sum(left_cone) / len(left_cone) if left_cone else 3.0
        min_front = min(front_cone) if front_cone else 3.0
        
        twist = Twist()
        
        if min_front < self.lookahead_distance:
            # Brake and turn away from the closer wall
            twist.linear.x = 0.5
            if avg_right < avg_left:
                twist.angular.z = self.steering_gain  # Turn Left
            else:
                twist.angular.z = -self.steering_gain # Turn Right
            
            # Reset PID state on sharp obstacle avoidance
            self.prev_error = 0.0
            
            self.get_logger().debug(f"Obstacle ahead ({min_front:.2f}m)! Evading.")
        else:
            # Center-line PID tracking
            error = avg_left - avg_right
            derivative = error - self.prev_error
            self.prev_error = error
            
            control = self.kp * error + self.kd * derivative
            control = max(-2.5, min(2.5, control))
            
            # Dynamic speed adjustment based on steering effort
            speed = self.max_forward_speed * max(0.5, 1.0 - 0.3 * abs(control))
            
            twist.linear.x = speed
            twist.angular.z = control
            
        self.get_logger().info(f"Publishing Speed: {twist.linear.x}, Turn: {twist.angular.z}")
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutopilotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
