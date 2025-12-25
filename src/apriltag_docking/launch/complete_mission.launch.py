#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the current workspace directory
    workspace_dir = '/root/docking_ws'
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description='World file to load'
        ),
        
        DeclareLaunchArgument(
            'record_video',
            default_value='true',
            description='Whether to record camera video'
        ),
        
        DeclareLaunchArgument(
            'record_rosbag',
            default_value='true', 
            description='Whether to record rosbag'
        ),
        
        LogInfo(msg="Starting Gazebo simulation..."),
        
        # Launch Gazebo
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen',
            name='gazebo'
        ),
        
        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            arguments=[
                '-entity', 'mecanum_robot',
                '-file', f'{workspace_dir}/src/apriltag_docking/urdf/mecanum_robot.urdf.xacro',
                '-x', '0.0',
                '-y', '0.0', 
                '-z', '0.1'
            ],
            output='screen'
        ),
        
        # Mission Controller
        Node(
            package='apriltag_docking',
            executable='mission_controller.py',
            name='mission_controller',
            output='screen',
            parameters=[{
                'mission_file': f'{workspace_dir}/src/apriltag_docking/config/apriltag_config.yaml'
            }]
        ),
        
        # AprilTag Follower
        Node(
            package='apriltag_docking',
            executable='apriltag_follower.py', 
            name='apriltag_follower',
            output='screen'
        ),
        
        # Camera Recorder (conditional)
        Node(
            package='apriltag_docking',
            executable='camera_recorder.py',
            name='camera_recorder',
            output='screen',
            condition=LaunchConfiguration('record_video')
        ),
        
        # ROS bag recording (conditional)
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '/camera/image_raw',
                '/cmd_vel',
                '/odom', 
                '/target_tag',
                '/tag_reached',
                '/lift_position',
                '/tf',
                '/tf_static',
                '-o', f'{workspace_dir}/complete_mission_$(date +%Y%m%d_%H%M%S)'
            ],
            output='screen',
            name='rosbag_record',
            condition=LaunchConfiguration('record_rosbag')
        ),
        
        LogInfo(msg="All nodes started! Mission will begin in 5 seconds...")
    ])