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
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.target_sub = self.create_subscription(
            Int32, '/target_tag', self.target_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reached_pub = self.create_publisher(Bool, '/tag_reached', 10)
        
        self.detector = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        self.get_logger().info('AprilTag Follower Started')

    def target_callback(self, msg):
        self.current_target_tag = msg.data
        self.reached_target = False
        self.get_logger().info(f'New target tag: {self.current_target_tag}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.detector, parameters=self.parameters)
            
            twist = Twist()
            
            if ids is not None and len(ids) > 0:
                target_found = False
                for i, tag_id in enumerate(ids):
                    if tag_id[0] == self.current_target_tag:
                        target_found = True
                        self.tag_detected = True
                        
                        corner = corners[i][0]
                        cx = np.mean(corner[:, 0])
                        cy = np.mean(corner[:, 1])
                        
                        img_center_x = cv_image.shape[1] / 2
                        img_center_y = cv_image.shape[0] / 2
                        
                        tag_width = np.linalg.norm(corner[1] - corner[0])
                        
                        kp_angular = 0.003
                        kp_linear = 0.5
                        target_size = 200
                        
                        error_x = cx - img_center_x
                        angular_z = -kp_angular * error_x
                        
                        size_error = target_size - tag_width
                        linear_x = kp_linear * (size_error / target_size)
                        
                        linear_x = max(-0.3, min(0.3, linear_x))
                        angular_z = max(-0.5, min(0.5, angular_z))
                        
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
                        break
                
                if not target_found:
                    twist.angular.z = 0.3
            else:
                twist.angular.z = 0.3
            
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
