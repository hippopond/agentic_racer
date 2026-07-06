import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from nav_msgs.msg import Odometry
import json
import os
import subprocess
import time
import math

class RefereeNode(Node):
    def __init__(self):
        super().__init__('referee_node')
        self.get_logger().info("Referee Node initialized! Waiting for the race to start...")
        
        # State Tracking
        self.laps_completed = 0
        self.crashes = 0
        self.start_time = self.get_clock().now()
        self.crashed = False
        
        # Speed and Distance Tracking
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0
        self.lap_distance = 0.0
        self.current_speed = 0.0
        
        # Lap Tracking State Machine
        self.expected_crossing_dir = None
        self.lap_start_time = self.get_clock().now()
        self.lap_times = []
        
        self.contact_sub = self.create_subscription(Contacts, '/bumper', self.contact_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        if self.crashed:
            return
            
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Calculate current speed from twist
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.hypot(vx, vy)
        
        if self.prev_x is None:
            self.prev_x = x
            self.prev_y = y
            return
            
        dist_step = math.hypot(x - self.prev_x, y - self.prev_y)
        self.total_distance += dist_step
        self.lap_distance += dist_step
        
        # Track-agnostic lap counting: crossing the Y axis (X=0) where Y > 0
        if (self.prev_x < 0 and x >= 0) or (self.prev_x >= 0 and x < 0):
            if y > 0.0 and abs(x) < 2.0:
                if self.lap_distance > 5.0:
                    crossing_dir = 1 if x >= self.prev_x else -1
                    
                    if self.expected_crossing_dir is None:
                        self.expected_crossing_dir = crossing_dir
                        
                    if crossing_dir == self.expected_crossing_dir:
                        current_time = self.get_clock().now()
                        lap_time = (current_time - self.lap_start_time).nanoseconds / 1e9
                        
                        # Prevent false triggers if time is too short
                        if lap_time > 5.0:
                            self.lap_times.append(lap_time)
                            self.laps_completed += 1
                            
                            total_elapsed = (current_time - self.start_time).nanoseconds / 1e9
                            avg_speed = self.total_distance / total_elapsed if total_elapsed > 0 else 0.0
                            
                            self.get_logger().info(f"🏁 LAP {self.laps_completed} COMPLETED! Time: {lap_time:.2f}s | Curr Speed: {self.current_speed:.2f} m/s | Avg Speed: {avg_speed:.2f} m/s")
                            
                            # Reset for next lap
                            self.lap_start_time = current_time
                            self.lap_distance = 0.0

        self.prev_x = x
        self.prev_y = y

    def contact_callback(self, msg):
        contacts_list = getattr(msg, 'contacts', getattr(msg, 'contact', []))
        if len(contacts_list) > 0 and not self.crashed:
            self.get_logger().error("CRASH DETECTED!")
            self.crashes = 1
            self.crashed = True
            self.generate_report()
            
            # Gracefully kill ROS 2 nodes (including Gazebo)
            subprocess.run(['killall', '-9', 'ruby'])
            subprocess.run(['killall', '-9', 'gz'])
            os._exit(0)

    def generate_report(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        avg_speed = self.total_distance / elapsed_time if elapsed_time > 0 else 0.0
        
        report = {
            "laps_completed": self.laps_completed,
            "lap_times_sec": self.lap_times,
            "crashes": self.crashes,
            "total_elapsed_time_sec": elapsed_time,
            "total_distance_m": self.total_distance,
            "average_speed_m_s": avg_speed
        }
        
        report_path = os.path.join(os.getcwd(), 'report.json')
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
