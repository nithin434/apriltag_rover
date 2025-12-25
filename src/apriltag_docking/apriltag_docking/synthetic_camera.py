#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32, Bool, Float64
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
import time

class SyntheticCameraNode(Node):
    def __init__(self):
        super().__init__('synthetic_camera')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Subscribers for mission data
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.target_tag_sub = self.create_subscription(
            Int32, '/target_tag', self.target_tag_callback, 10)
        self.tag_reached_sub = self.create_subscription(
            Bool, '/tag_reached', self.tag_reached_callback, 10)
        self.lift_pos_sub = self.create_subscription(
            Float64, '/lift_position', self.lift_position_callback, 10)
        
        # State variables
        self.bridge = CvBridge()
        self.current_target = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_angle = 0.0
        self.lift_position = 0.0
        self.tag_reached = False
        self.mission_phase = "IDLE"
        
        # Camera parameters
        self.image_width = 640
        self.image_height = 480
        
        # Timer for publishing camera frames
        self.timer = self.create_timer(1.0/30.0, self.publish_camera_frame)  # 30 FPS
        
        self.get_logger().info("Synthetic Camera Node Started - Publishing camera frames for Foxglove")
        
    def cmd_vel_callback(self, msg):
        """Update robot position based on velocity commands"""
        dt = 0.1  # Time step
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Simple kinematic model
        self.robot_angle += angular_vel * dt
        self.robot_x += linear_vel * np.cos(self.robot_angle) * dt
        self.robot_y += linear_vel * np.sin(self.robot_angle) * dt
        
    def target_tag_callback(self, msg):
        """Update current mission target"""
        self.current_target = msg.data
        
        # Update mission phase based on target
        if self.current_target == 1:
            self.mission_phase = "GO_TO_DOCK1"
        elif self.current_target == 2:
            self.mission_phase = "LIFT_UP"
        elif self.current_target == 3:
            self.mission_phase = "GO_TO_DOCK2"
        elif self.current_target == 4:
            self.mission_phase = "LIFT_DOWN"
        elif self.current_target == 5:
            self.mission_phase = "GO_TO_CHARGER"
        else:
            self.mission_phase = "IDLE"
            
    def tag_reached_callback(self, msg):
        """Update tag reached status"""
        self.tag_reached = msg.data
        
    def lift_position_callback(self, msg):
        """Update lift position"""
        self.lift_position = msg.data
        
    def create_synthetic_frame(self):
        """Create a synthetic camera frame showing the mission environment"""
        # Create base frame (dark blue background)
        frame = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        frame[:, :] = [40, 40, 100]  # Dark blue background
        
        # Draw environment elements
        self.draw_environment(frame)
        
        # Draw robot position
        self.draw_robot(frame)
        
        # Draw mission information
        self.draw_mission_info(frame)
        
        return frame
    
    def draw_environment(self, frame):
        """Draw docking stations and environment"""
        h, w = frame.shape[:2]
        
        # Docking Station 1 (right side)
        dock1_x, dock1_y = int(w * 0.8), int(h * 0.5)
        cv2.rectangle(frame, (dock1_x - 30, dock1_y - 40), 
                     (dock1_x + 30, dock1_y + 40), (0, 0, 255), 2)
        cv2.putText(frame, "DOCK 1", (dock1_x - 25, dock1_y - 45), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Docking Station 2 (left side)
        dock2_x, dock2_y = int(w * 0.2), int(h * 0.5)
        cv2.rectangle(frame, (dock2_x - 30, dock2_y - 40), 
                     (dock2_x + 30, dock2_y + 40), (0, 0, 255), 2)
        cv2.putText(frame, "DOCK 2", (dock2_x - 25, dock2_y - 45), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Charging Station (top center)
        charge_x, charge_y = int(w * 0.5), int(h * 0.2)
        cv2.rectangle(frame, (charge_x - 25, charge_y - 25), 
                     (charge_x + 25, charge_y + 25), (0, 255, 0), 2)
        cv2.putText(frame, "CHARGE", (charge_x - 30, charge_y - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        # Highlight current target
        if self.current_target == 1 or self.current_target == 2:
            cv2.circle(frame, (dock1_x, dock1_y), 50, (0, 255, 255), 3)
        elif self.current_target == 3 or self.current_target == 4:
            cv2.circle(frame, (dock2_x, dock2_y), 50, (0, 255, 255), 3)
        elif self.current_target == 5:
            cv2.circle(frame, (charge_x, charge_y), 35, (0, 255, 255), 3)
    
    def draw_robot(self, frame):
        """Draw robot at its current position"""
        h, w = frame.shape[:2]
        
        # Convert robot position to screen coordinates
        robot_screen_x = int(w * 0.5 + self.robot_x * 80)
        robot_screen_y = int(h * 0.5 - self.robot_y * 80)
        
        # Clamp to screen bounds
        robot_screen_x = max(30, min(w - 30, robot_screen_x))
        robot_screen_y = max(30, min(h - 30, robot_screen_y))
        
        # Draw robot body
        cv2.rectangle(frame, (robot_screen_x - 15, robot_screen_y - 10), 
                     (robot_screen_x + 15, robot_screen_y + 10), (255, 255, 0), -1)
        
        # Draw direction indicator
        end_x = int(robot_screen_x + 20 * np.cos(self.robot_angle))
        end_y = int(robot_screen_y - 20 * np.sin(self.robot_angle))
        cv2.arrowedLine(frame, (robot_screen_x, robot_screen_y), 
                       (end_x, end_y), (255, 255, 255), 2)
        
        # Draw lift indicator
        if self.lift_position > 0.1:
            cv2.rectangle(frame, (robot_screen_x - 5, robot_screen_y - 25), 
                         (robot_screen_x + 5, robot_screen_y - 15), (255, 0, 255), -1)
            cv2.putText(frame, "LIFT", (robot_screen_x - 15, robot_screen_y - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 255), 1)
        
    def draw_mission_info(self, frame):
        """Draw mission status information"""
        h, w = frame.shape[:2]
        
        # Mission phase
        cv2.putText(frame, f"Mission: {self.mission_phase}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Target information
        cv2.putText(frame, f"Target: {self.current_target}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        
        # Robot position
        cv2.putText(frame, f"Pos: ({self.robot_x:.1f}, {self.robot_y:.1f})", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Lift position
        cv2.putText(frame, f"Lift: {self.lift_position:.1f}m", (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        # Status
        status_color = (0, 255, 0) if self.tag_reached else (255, 255, 255)
        status_text = "REACHED" if self.tag_reached else "MOVING"
        cv2.putText(frame, f"Status: {status_text}", (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
        
        # Timestamp
        cv2.putText(frame, f"Time: {self.get_clock().now().to_msg().sec}", 
                   (w - 150, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
        
    def publish_camera_frame(self):
        """Publish synthetic camera frame and camera info"""
        # Create synthetic frame
        frame = self.create_synthetic_frame()
        
        # Convert to ROS Image message
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera_link"
            
            # Publish image
            self.image_pub.publish(image_msg)
            
            # Publish camera info
            camera_info = CameraInfo()
            camera_info.header = image_msg.header
            camera_info.width = self.image_width
            camera_info.height = self.image_height
            camera_info.distortion_model = "plumb_bob"
            
            # Simple camera matrix (approximate)
            camera_info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
            camera_info.p = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            
            self.camera_info_pub.publish(camera_info)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing camera frame: {e}")

def main():
    rclpy.init()
    node = SyntheticCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()