import sys
import os
import time
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from agentic_racer.autopilot_node import AutopilotNode

class TestAutopilotNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = AutopilotNode()
        
    def tearDown(self):
        self.node.destroy_node()

    def test_node_initialization(self):
        """T3.1: The Python script must initialize the ROS 2 node without crashing."""
        self.assertEqual(self.node.get_name(), 'autopilot_node')

    def test_subscription(self):
        """T3.2: The node must successfully subscribe to the LaserScan message type on the /scan topic."""
        subs = self.node.subscriptions
        found = False
        for sub in subs:
            if sub.topic_name == '/scan' and sub.msg_type == LaserScan:
                found = True
        self.assertTrue(found, "Did not find /scan subscription with LaserScan type")

    def test_publication(self):
        """T3.3: The node must successfully publish Twist messages to /cmd_vel."""
        pubs = self.node.publishers
        found = False
        for pub in pubs:
            if pub.topic_name == '/cmd_vel' and pub.msg_type == Twist:
                found = True
        self.assertTrue(found, "Did not find /cmd_vel publisher with Twist type")

    def test_brain_in_a_jar(self):
        """T3.4: When injected with mock LiDAR data indicating a wall 0.1m ahead, 
        the node must publish a braking/turning command within 10 milliseconds."""
        
        published_msgs = []
        def mock_publish(msg):
            published_msgs.append(msg)
            
        self.node.publisher.publish = mock_publish
        
        scan = LaserScan()
        scan.ranges = [0.1] * 360
        
        start_time = time.time()
        self.node.scan_callback(scan)
        end_time = time.time()
        
        execution_time_ms = (end_time - start_time) * 1000
        
        self.assertTrue(len(published_msgs) > 0, "Node did not publish a message")
        msg = published_msgs[-1]
        
        self.assertTrue(execution_time_ms <= 10.0, f"Execution time {execution_time_ms}ms exceeded 10ms")
        self.assertEqual(msg.linear.x, 0.0, "Node did not brake (linear.x should be 0.0)")
        self.assertTrue(msg.angular.z > 0.0, "Node did not turn (angular.z should be > 0.0)")

    def test_latency(self):
        """T3.5: The main control loop execution time must be under 50ms to ensure real-time physics reactivity."""
        self.node.publisher.publish = lambda msg: None
        
        scan = LaserScan()
        scan.ranges = [1.0] * 360
        
        start_time = time.time()
        self.node.scan_callback(scan)
        end_time = time.time()
        
        execution_time_ms = (end_time - start_time) * 1000
        self.assertTrue(execution_time_ms <= 50.0, f"Execution time {execution_time_ms}ms exceeded 50ms")

if __name__ == '__main__':
    unittest.main()
