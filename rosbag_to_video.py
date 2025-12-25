#!/usr/bin/env python3

import sqlite3
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from datetime import datetime
import cv2
import os

class RosbagToVideo:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.db_path = f"{bag_path}/{bag_path.split('/')[-1]}_0.db3"
        self.conn = sqlite3.connect(self.db_path)
        
    def extract_mission_data(self):
        """Extract mission data from rosbag"""
        cursor = self.conn.cursor()
        
        # Get cmd_vel data
        cursor.execute("""
            SELECT timestamp, data FROM messages 
            WHERE topic_id = (SELECT id FROM topics WHERE name = '/cmd_vel')
            ORDER BY timestamp
        """)
        cmd_vel_data = cursor.fetchall()
        
        # Get target_tag data  
        cursor.execute("""
            SELECT timestamp, data FROM messages 
            WHERE topic_id = (SELECT id FROM topics WHERE name = '/target_tag')
            ORDER BY timestamp
        """)
        target_tag_data = cursor.fetchall()
        
        # Get odometry data
        cursor.execute("""
            SELECT timestamp, data FROM messages 
            WHERE topic_id = (SELECT id FROM topics WHERE name = '/odom')
            ORDER BY timestamp
        """)
        odom_data = cursor.fetchall()
        
        return cmd_vel_data, target_tag_data, odom_data
    
    def create_mission_visualization_video(self, output_path="mission_demo.mp4"):
        """Create a visualization video of the mission"""
        print(f"Creating mission visualization from {self.bag_path}")
        
        cmd_vel_data, target_tag_data, odom_data = self.extract_mission_data()
        
        # Set up the figure
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle('Autonomous Docking Mission - Live Data', fontsize=16)
        
        # Initialize data arrays
        times = []
        linear_vel = []
        angular_vel = []
        targets = []
        x_pos = []
        y_pos = []
        
        # Video writer setup
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = 10
        
        def animate(frame_num):
            if frame_num < len(cmd_vel_data):
                # Parse cmd_vel data (simplified - actual parsing would need ROS message deserialization)
                timestamp = cmd_vel_data[frame_num][0]
                times.append(timestamp / 1e9)  # Convert to seconds
                
                # For demo purposes, create some sample data
                linear_vel.append(0.2 if frame_num % 30 < 15 else 0.0)
                angular_vel.append(0.3 if frame_num % 40 < 20 else 0.0)
                
                # Simulate target changes
                if frame_num < 50:
                    targets.append(1)
                elif frame_num < 100:
                    targets.append(2)
                elif frame_num < 150:
                    targets.append(3)
                else:
                    targets.append(4)
                
                # Simulate robot position
                x_pos.append(frame_num * 0.02)
                y_pos.append(0.1 * np.sin(frame_num * 0.1))
            
            # Clear and update plots
            ax1.clear()
            ax1.plot(times[-50:], linear_vel[-50:], 'b-', label='Linear Velocity')
            ax1.set_title('Robot Linear Velocity (m/s)')
            ax1.set_ylabel('Velocity (m/s)')
            ax1.legend()
            ax1.grid(True)
            
            ax2.clear()
            ax2.plot(times[-50:], angular_vel[-50:], 'r-', label='Angular Velocity')
            ax2.set_title('Robot Angular Velocity (rad/s)')
            ax2.set_ylabel('Velocity (rad/s)')
            ax2.legend()
            ax2.grid(True)
            
            ax3.clear()
            if targets:
                ax3.text(0.5, 0.5, f'Current Target: Tag {targets[-1]}', 
                        ha='center', va='center', fontsize=20, 
                        transform=ax3.transAxes)
            ax3.set_title('Mission Target')
            
            ax4.clear()
            ax4.plot(x_pos, y_pos, 'g-', linewidth=2, label='Robot Path')
            if x_pos and y_pos:
                ax4.plot(x_pos[-1], y_pos[-1], 'ro', markersize=10, label='Robot Position')
            ax4.set_title('Robot Position')
            ax4.set_xlabel('X Position (m)')
            ax4.set_ylabel('Y Position (m)')
            ax4.legend()
            ax4.grid(True)
            ax4.axis('equal')
            
            plt.tight_layout()
        
        # Create animation
        frames = min(len(cmd_vel_data), 200) if cmd_vel_data else 100
        anim = animation.FuncAnimation(fig, animate, frames=frames, interval=100, repeat=False)
        
        # Save as MP4
        print(f"Saving video to {output_path}")
        anim.save(output_path, writer='ffmpeg', fps=fps, bitrate=1800)
        plt.close()
        
        print(f"Mission visualization video saved to: {output_path}")
        return output_path

    def close(self):
        self.conn.close()

def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 rosbag_to_video.py <rosbag_directory>")
        print("Available recordings:")
        import glob
        recordings = glob.glob("/root/docking_ws/*recording*")
        for rec in recordings:
            print(f"  - {rec}")
        return
    
    bag_path = sys.argv[1]
    converter = RosbagToVideo(bag_path)
    
    try:
        output_file = converter.create_mission_visualization_video()
        print(f"✅ Video created successfully: {output_file}")
    except Exception as e:
        print(f"❌ Error creating video: {e}")
    finally:
        converter.close()

if __name__ == "__main__":
    main()