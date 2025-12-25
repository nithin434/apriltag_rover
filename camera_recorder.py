#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import threading
import time

class CameraRecorder(Node):
    def __init__(self):
        super().__init__('camera_recorder')
        
        self.bridge = CvBridge()
        self.recording = False
        self.video_writer = None
        self.frame_count = 0
        self.frames_buffer = []
        self.max_buffer_size = 100
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Create recordings directory
        self.recordings_dir = "/root/docking_ws/camera_recordings"
        os.makedirs(self.recordings_dir, exist_ok=True)
        
        self.get_logger().info("Camera recorder initialized - waiting for camera data...")
        
        # Timer to check for images and start recording
        self.check_timer = self.create_timer(2.0, self.check_and_start_recording)
        self.recording_started = False
    
    def check_and_start_recording(self):
        """Check if we're getting camera data and start recording"""
        if not self.recording_started and self.frame_count > 0:
            self.start_recording()
            self.recording_started = True
    
    def start_recording(self):
        """Start video recording"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_filename = f"{self.recordings_dir}/mission_camera_{timestamp}.mp4"
        
        # Video codec and parameters
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fps = 30.0
        frame_size = (640, 480)  # Match camera resolution
        
        self.video_writer = cv2.VideoWriter(
            self.video_filename, 
            fourcc, 
            fps, 
            frame_size
        )
        
        self.recording = True
        self.get_logger().info(f"Started recording to: {self.video_filename}")
        
        # Write any buffered frames
        for frame in self.frames_buffer:
            self.video_writer.write(frame)
        self.frames_buffer.clear()
    
    def image_callback(self, msg):
        """Process camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Add mission info overlay
            timestamp = self.get_clock().now().to_msg()
            time_text = f"Time: {timestamp.sec}.{timestamp.nanosec//1000000:03d}"
            cv2.putText(cv_image, time_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add frame counter
            frame_text = f"Frame: {self.frame_count}"
            cv2.putText(cv_image, frame_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add mission title
            cv2.putText(cv_image, "AprilTag Autonomous Docking", (10, cv_image.shape[0] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            if self.recording and self.video_writer:
                # Write frame directly to video
                self.video_writer.write(cv_image)
            else:
                # Buffer frames before recording starts
                if len(self.frames_buffer) < self.max_buffer_size:
                    self.frames_buffer.append(cv_image.copy())
                else:
                    self.frames_buffer.pop(0)
                    self.frames_buffer.append(cv_image.copy())
            
            self.frame_count += 1
            
            if self.frame_count % 100 == 0:
                self.get_logger().info(f"Processed {self.frame_count} camera frames")
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def stop_recording(self):
        """Stop video recording"""
        if self.video_writer:
            self.video_writer.release()
            self.recording = False
            self.get_logger().info(f"Recording stopped. Saved {self.frame_count} frames to {self.video_filename}")
    
    def __del__(self):
        self.stop_recording()

def main():
    rclpy.init()
    recorder = CameraRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop_recording()
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()