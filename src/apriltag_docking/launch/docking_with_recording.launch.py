#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    pkg_name = 'apriltag_docking'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'camera_world.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mecanum_robot.urdf.xacro')
    
    # Process URDF
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # Gazebo with GUI for camera recording
    gzserver = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen',
        env={'DISPLAY': ':0'}  # Try to use display if available
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mecanum_robot',
                   '-topic', '/robot_description',
                   '-x', '0', '-y', '0', '-z', '0.15'],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Mission controller
    mission_controller = Node(
        package=pkg_name,
        executable='mission_controller',
        output='screen'
    )
    
    # AprilTag follower
    apriltag_follower = Node(
        package=pkg_name,
        executable='apriltag_follower',
        output='screen'
    )
    
    # Camera recorder (start after robot spawns)
    camera_recorder = TimerAction(
        period=10.0,  # Wait 10 seconds for camera to initialize
        actions=[
            Node(
                package='apriltag_docking',
                executable='camera_recorder.py',
                name='camera_recorder',
                output='screen'
            )
        ]
    )
    
    # Screen recording using ffmpeg to capture Gazebo window
    screen_record = TimerAction(
        period=15.0,  # Start recording after everything is set up
        actions=[
            ExecuteProcess(
                cmd=['ffmpeg', '-y', '-f', 'x11grab', '-r', '30', '-s', '1920x1080',
                     '-i', ':0.0', '-t', '60', '-vcodec', 'libx264', '-crf', '25',
                     '/root/docking_ws/gazebo_screen_recording.mp4'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gzserver,
        robot_state_pub,
        spawn_robot,
        mission_controller,
        apriltag_follower,
        camera_recorder,
        screen_record,
    ])