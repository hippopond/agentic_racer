import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ros_gz_interfaces.msg import Contacts
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
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
        
        # New Metrics
        self.wrong_way_events = 0
        self.high_pitch_events = 0
        self.stuck_events = 0
        
        self.max_pitch_deg = 0.0
        self.stuck_timer_start = None
        self.wrong_way_cooldown = 0
        
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
        
        # Telemetry
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel_test', self.cmd_callback, 10)
        self.current_steering = 0.0
        self.prev_steering = 0.0
        self.max_steering_jerk = 0.0
        self.telemetry_log = []
        self.last_telemetry_time = self.get_clock().now()
        self.waypoints = self.generate_waypoints()
        
        self.contact_sub = self.create_subscription(Contacts, '/bumper', self.contact_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def generate_waypoints(self):
        waypoints = []
        R = 4.5
        for i in range(200):
            t = -math.pi/2 + (i * 2 * math.pi / 200)
            waypoints.append((R * (math.cos(t) - 0.25 * math.cos(2*t)) + 3.0, R * math.sin(t)))
        return waypoints

    def cmd_callback(self, msg):
        self.current_steering = msg.angular.z

    def odom_callback(self, msg):
        if self.crashed:
            return
            
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        
        # 1. Pitch Calculation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        pitch_deg = math.degrees(abs(pitch))
        
        if pitch_deg > self.max_pitch_deg:
            self.max_pitch_deg = pitch_deg
            
        if pitch_deg > 10.0:
            self.high_pitch_events += 1
            if self.high_pitch_events % 50 == 0:
                self.get_logger().warn(f"High Pitch Detected! {pitch_deg:.1f} degrees")
        
        # 2. Wrong Way Calculation (Cross Product)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        global_x = x + 4.125
        global_y = y - 4.5
        cross_product = global_x * math.sin(yaw) - global_y * math.cos(yaw)
        
        current_time = self.get_clock().now()
        
        if cross_product < -0.5:
            # Cool down to prevent spamming (2 seconds)
            if current_time.nanoseconds - self.wrong_way_cooldown > 2e9:
                self.wrong_way_events += 1
                self.get_logger().warn("Wrong Way Detected by Referee!")
                self.wrong_way_cooldown = current_time.nanoseconds
                
        # 3. Telemetry Log (Roll, CTE, Steering Jerk)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_deg = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        
        cte = min(math.hypot(wx - global_x, wy - global_y) for wx, wy in self.waypoints)
        
        dt = (current_time - self.last_telemetry_time).nanoseconds / 1e9
        if dt > 0:
            jerk = abs(self.current_steering - self.prev_steering) / dt
            if jerk > self.max_steering_jerk:
                self.max_steering_jerk = jerk
        self.prev_steering = self.current_steering
        
        if dt >= 0.5:
            self.telemetry_log.append({
                "time_sec": (current_time - self.start_time).nanoseconds / 1e9,
                "x": global_x,
                "y": global_y,
                "yaw_deg": math.degrees(yaw),
                "speed": self.current_speed,
                "steering": self.current_steering,
                "cte": cte,
                "pitch_deg": pitch_deg,
                "roll_deg": roll_deg
            })
            self.last_telemetry_time = current_time
        
        # 4. Stuck Detection
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.hypot(vx, vy)
        
        if self.current_speed < 0.1:
            if self.stuck_timer_start is None:
                self.stuck_timer_start = current_time
            else:
                stuck_duration = (current_time - self.stuck_timer_start).nanoseconds / 1e9
                if stuck_duration > 3.0:
                    self.stuck_events += 1
                    self.get_logger().warn("Robot is STUCK!")
                    self.stuck_timer_start = current_time # Reset to ping every 3 sec
        else:
            self.stuck_timer_start = None
        
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
            
            # Gracefully kill ROS 2 nodes (including Gazebo)
            subprocess.run(['killall', '-9', 'ruby'])
            subprocess.run(['killall', '-9', 'gz'])
            raise ExternalShutdownException()

    def generate_report(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        avg_speed = self.total_distance / elapsed_time if elapsed_time > 0 else 0.0
        
        report = {
            "laps_completed": self.laps_completed,
            "lap_times_sec": self.lap_times,
            "crashes": self.crashes,
            "wrong_way_events": self.wrong_way_events,
            "high_pitch_events": self.high_pitch_events,
            "max_pitch_deg": self.max_pitch_deg,
            "max_steering_jerk": self.max_steering_jerk,
            "stuck_events": self.stuck_events,
            "total_elapsed_time_sec": elapsed_time,
            "total_distance_m": self.total_distance,
            "average_speed_m_s": avg_speed,
            "telemetry": self.telemetry_log
        }
        
        report_path = os.path.join(os.getcwd(), 'report.json')
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=4)
        self.get_logger().info(f"\n======================================\nRace finished. Evaluation report saved to {report_path}\n======================================")

def main(args=None):
    rclpy.init(args=args)
    node = RefereeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.generate_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
