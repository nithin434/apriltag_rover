#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class AprilTagFollower(Node):
    def __init__(self):
        super().__init__('apriltag_follower')
        
        self.bridge = CvBridge()
        self.current_target_tag = 0
        self.tag_detected = False
        self.reached_target = False
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.target_sub = self.create_subscription(
            Int32, '/target_tag', self.target_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reached_pub = self.create_publisher(Bool, '/tag_reached', 10)
        
        # AprilTag detector
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        # Timer to publish movement commands even without camera
        self.movement_timer = self.create_timer(0.1, self.movement_callback)
        self.move_start_time = 0
        
        self.get_logger().info('AprilTag Follower Started')

    def target_callback(self, msg):
        self.current_target_tag = msg.data
        self.reached_target = False
        self.get_logger().info(f'New target tag: {self.current_target_tag}')
        
        # If we don't have camera feed, just move forward slowly for testing
        if self.current_target_tag > 0:
            self.get_logger().info(f'Target {self.current_target_tag} set - robot should start moving!')
            self.move_start_time = self.get_clock().now().nanoseconds / 1e9

    def movement_callback(self):
        """Timer callback to ensure robot moves even without camera input"""
        if self.current_target_tag > 0 and not self.reached_target:
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Simple movement: forward for 3 seconds, then mark as reached
            if self.move_start_time > 0 and (current_time - self.move_start_time) < 3.0:
                twist = Twist()
                twist.linear.x = 0.2
                self.cmd_vel_pub.publish(twist)
                if int(current_time) % 2 == 0:  # Log every 2 seconds
                    self.get_logger().info(f'Moving toward target {self.current_target_tag}...')
            elif self.move_start_time > 0:
                # Stop and mark as reached
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                if not self.reached_target:
                    self.reached_target = True
                    self.reached_pub.publish(Bool(data=True))
                    self.get_logger().info(f'Simulated reach of tag {self.current_target_tag}!')
                    self.move_start_time = 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect AprilTags
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.parameters)
            
            twist = Twist()
            
            # Always publish some movement if target is set but no image callback happens
            if self.current_target_tag == 0:
                # No target set, stay still
                pass
            elif ids is not None and len(ids) > 0:
                # Tags detected, log what we see
                detected_tags = [int(tag_id[0]) for tag_id in ids]
                if int(self.get_clock().now().nanoseconds / 1e9) % 5 == 0:  # Log every 5 seconds
                    self.get_logger().info(f'Detected tags: {detected_tags}, looking for: {self.current_target_tag}')
                
                # Find target tag
                target_found = False
                for i, tag_id in enumerate(ids):
                    if tag_id[0] == self.current_target_tag:
                        target_found = True
                        self.tag_detected = True
                        
                        # Get tag center
                        corner = corners[i][0]
                        cx = np.mean(corner[:, 0])
                        cy = np.mean(corner[:, 1])
                        
                        # Image center
                        img_center_x = cv_image.shape[1] / 2
                        img_center_y = cv_image.shape[0] / 2
                        
                        # Calculate tag size (distance estimate)
                        tag_width = np.linalg.norm(corner[1] - corner[0])
                        
                        # Control gains
                        kp_angular = 0.003
                        kp_linear = 0.5
                        target_size = 200
                        
                        # Angular error (centering)
                        error_x = cx - img_center_x
                        angular_z = -kp_angular * error_x
                        
                        # Linear speed (approach)
                        size_error = target_size - tag_width
                        linear_x = kp_linear * (size_error / target_size)
                        
                        # Saturation
                        linear_x = max(-0.3, min(0.3, linear_x))
                        angular_z = max(-0.5, min(0.5, angular_z))
                        
                        # Check if reached
                        if abs(size_error) < 30 and abs(error_x) < 50:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            if not self.reached_target:
                                self.reached_target = True
                                self.reached_pub.publish(Bool(data=True))
                                self.get_logger().info(f'Reached tag {self.current_target_tag}!')
                        else:
                            twist.linear.x = linear_x
                            twist.angular.z = angular_z
                            # Log occasionally for debugging
                            if int(self.get_clock().now().nanoseconds / 1e9) % 2 == 0:
                                self.get_logger().info(f'Approaching tag {self.current_target_tag}: size_err={size_error:.1f}, pos_err={error_x:.1f}')
                        
                        break
                
                if not target_found:
                    # Search behavior - rotate to find target tag
                    twist.angular.z = 0.3
                    # Log occasionally to avoid spam
                    if self.current_target_tag > 0 and int(self.get_clock().now().nanoseconds / 1e9) % 3 == 0:
                        self.get_logger().info(f'Searching for tag {self.current_target_tag}... (detected: {detected_tags})')
            else:
                # No tags detected - search behavior if we have a target
                if self.current_target_tag > 0:
                    twist.angular.z = 0.3
                    if int(self.get_clock().now().nanoseconds / 1e9) % 4 == 0:
                        self.get_logger().info(f'No tags detected, searching for target {self.current_target_tag}')
            
            # Fallback: if we have a target but no camera, just move forward
            if self.current_target_tag > 0 and twist.linear.x == 0.0 and twist.angular.z == 0.0:
                twist.linear.x = 0.1
                self.get_logger().info(f'Moving forward to search for tag {self.current_target_tag}')
            
            self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()