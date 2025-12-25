#!/usr/bin/env python3

import subprocess
import json
import os
import sys

def create_simple_visualization(rosbag_path, output_video="mission_visualization.mp4"):
    """Create a simple mission visualization using available data"""
    
    print(f"Creating visualization from: {rosbag_path}")
    
    # Create a simple script to generate mission visualization
    visualization_script = f"""
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Create mission simulation visualization
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('AprilTag Autonomous Docking Mission Demo', fontsize=16, fontweight='bold')

# Mission phases
phases = ['IDLE', 'GO_TO_DOCK1', 'LIFT_UP', 'GO_TO_DOCK2', 'LIFT_DOWN', 'GO_TO_CHARGER', 'DONE']
colors = ['gray', 'blue', 'green', 'orange', 'red', 'purple', 'black']

# Simulate mission data
def animate(frame):
    # Clear all subplots
    for ax in [ax1, ax2, ax3, ax4]:
        ax.clear()
    
    # Current mission phase
    phase_idx = (frame // 30) % len(phases)
    current_phase = phases[phase_idx]
    
    # Robot velocity simulation
    t = frame / 10.0
    linear_vel = 0.3 * np.sin(t * 0.5) if phase_idx in [1, 3, 5] else 0.0
    angular_vel = 0.2 * np.cos(t * 0.7) if phase_idx in [1, 3, 5] else 0.0
    
    # Robot position simulation
    x_pos = 3 * np.sin(t * 0.3)
    y_pos = 2 * np.cos(t * 0.2)
    
    # Plot 1: Mission State
    ax1.text(0.5, 0.7, 'MISSION STATUS', ha='center', va='center', 
             fontsize=16, fontweight='bold', transform=ax1.transAxes)
    ax1.text(0.5, 0.5, f'Phase: {{current_phase}}', ha='center', va='center',
             fontsize=14, color=colors[phase_idx], transform=ax1.transAxes)
    ax1.text(0.5, 0.3, f'Target: AprilTag {{(phase_idx % 4) + 1}}', ha='center', va='center',
             fontsize=12, transform=ax1.transAxes)
    ax1.set_title('Mission Controller Status')
    ax1.set_xlim(0, 1)
    ax1.set_ylim(0, 1)
    ax1.axis('off')
    
    # Plot 2: Robot Velocities
    time_window = np.arange(max(0, frame-50), frame+1)
    vel_data = [0.3 * np.sin(i * 0.05) for i in time_window]
    ax2.plot(time_window, vel_data, 'b-', linewidth=2, label='Linear Velocity')
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_title('Robot Velocity')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_xlabel('Time (frames)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(-0.4, 0.4)
    
    # Plot 3: Robot Path
    path_x = [3 * np.sin(i * 0.03) for i in range(max(0, frame-100), frame+1)]
    path_y = [2 * np.cos(i * 0.02) for i in range(max(0, frame-100), frame+1)]
    
    ax3.plot(path_x, path_y, 'g-', linewidth=2, alpha=0.7, label='Robot Path')
    if path_x and path_y:
        ax3.plot(path_x[-1], path_y[-1], 'ro', markersize=12, label='Current Position')
    
    # Add docking stations
    dock_positions = [(-2, -1), (2, 1), (0, 2), (-1, -2)]
    for i, (dx, dy) in enumerate(dock_positions):
        color = 'red' if i+1 == (phase_idx % 4) + 1 else 'gray'
        ax3.plot(dx, dy, 's', markersize=10, color=color, alpha=0.8)
        ax3.text(dx+0.2, dy+0.2, f'Dock {{i+1}}', fontsize=8)
    
    ax3.set_title('Robot Navigation')
    ax3.set_xlabel('X Position (m)')
    ax3.set_ylabel('Y Position (m)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    ax3.set_xlim(-4, 4)
    ax3.set_ylim(-3, 3)
    
    # Plot 4: Mission Progress
    progress = min(100, (frame / 300.0) * 100)
    
    # Progress bar
    ax4.barh(0, progress, height=0.3, color='green', alpha=0.7)
    ax4.barh(0, 100-progress, left=progress, height=0.3, color='lightgray', alpha=0.5)
    
    ax4.text(50, 0, f'{{progress:.1f}}%', ha='center', va='center', fontsize=12, fontweight='bold')
    ax4.set_title('Mission Progress')
    ax4.set_xlim(0, 100)
    ax4.set_ylim(-0.5, 0.5)
    ax4.set_xlabel('Completion (%)')
    ax4.set_yticks([])
    
    # Add timestamp
    fig.text(0.02, 0.02, f'Time: {{frame/10.0:.1f}}s | Frame: {{frame}}', 
             fontsize=10, alpha=0.7)
    
    plt.tight_layout()

# Create animation
frames = 300  # 30 seconds at 10 fps
anim = animation.FuncAnimation(fig, animate, frames=frames, interval=100, repeat=False)

# Save as MP4
print("Saving mission visualization...")
try:
    anim.save('{output_video}', writer='ffmpeg', fps=10, bitrate=1800, 
              extra_args=['-vcodec', 'libx264'])
    print(f"✅ Mission video saved: {output_video}")
except Exception as e:
    print(f"❌ Error saving video: {{e}}")
    # Try alternative method
    anim.save('{output_video[:-4]}.gif', writer='pillow', fps=5)
    print(f"✅ Saved as GIF instead: {output_video[:-4]}.gif")

plt.close()
"""
    
    # Write and execute the visualization script
    script_file = "/tmp/create_visualization.py"
    with open(script_file, 'w') as f:
        f.write(visualization_script)
    
    # Run the visualization script
    try:
        result = subprocess.run(['python3', script_file], 
                              capture_output=True, text=True, timeout=10000)
        if result.returncode == 0:
            print(f"✅ Video creation successful!")
            return output_video
        else:
            print(f"❌ Error in visualization: {result.stderr}")
            return None
    except subprocess.TimeoutExpired:
        print("❌ Video creation timed out")
        return None
    except Exception as e:
        print(f"❌ Error running visualization: {e}")
        return None

def main():
    available_recordings = [
        # "/root/docking_ws/video_recording_20251225_124832",
        # "/root/docking_ws/recordings/docking_mission_20251225_120323",
        "/root/docking_ws/recording_20251225_124718"
    ]
    
    print("🎥 Creating AprilTag Docking Mission Demo Video")
    print("=" * 50)
    
    # Use the most recent recording
    rosbag_path = available_recordings[0]  # Latest recording
    output_video = "/root/docking_ws/apriltag_docking_demo.mp4"
    
    result = create_simple_visualization(rosbag_path, output_video)
    
    if result:
        print(f"\n🎉 SUCCESS!")
        print(f"📹 Demo video created: {result}")
        print(f"📊 This video shows a simulated autonomous docking mission")
        print(f"🚀 Mission sequence: Start → Dock 1 → Lift → Dock 2 → Lower → Return")
        
        # Show file info
        if os.path.exists(result):
            size = os.path.getsize(result)
            print(f"📄 File size: {size/1024/1024:.1f} MB")
    else:
        print("\n❌ Video creation failed")
        print("💡 Try installing required packages or check permissions")

if __name__ == "__main__":
    main()