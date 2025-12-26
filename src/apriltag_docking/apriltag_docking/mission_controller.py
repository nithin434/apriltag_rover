import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float64
from std_srvs.srv import Empty
import time

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # Declare and load parameters
        self.declare_parameter('enable_tag_following', True)
        self.declare_parameter('enable_background_camera', False)
        self.declare_parameter('mission_start_delay', 3.0)
        self.declare_parameter('lift_up_duration', 3.0)
        self.declare_parameter('lift_down_duration', 3.0)
        self.declare_parameter('tag_timeout', 30.0)
        self.declare_parameter('dock1_tags', [1, 2])
        self.declare_parameter('dock2_tags', [3, 4])
        self.declare_parameter('charger_tags', [5, 0])
        
        # Load parameters
        self.enable_tag_following = self.get_parameter('enable_tag_following').value
        self.enable_camera = self.get_parameter('enable_background_camera').value
        self.mission_delay = self.get_parameter('mission_start_delay').value
        self.lift_up_time = self.get_parameter('lift_up_duration').value
        self.lift_down_time = self.get_parameter('lift_down_duration').value
        self.tag_timeout = self.get_parameter('tag_timeout').value
        
        # State machine states
        self.IDLE = 0
        self.GO_TO_DOCK1 = 1
        self.LIFT_UP = 2
        self.GO_TO_DOCK2 = 3
        self.LIFT_DOWN = 4
        self.GO_TO_CHARGER = 5
        self.DONE = 6
        
        self.state = self.IDLE
        self.tag_reached = False
        self.current_timestamp = time.time()
        
        # Publishers
        self.target_tag_pub = self.create_publisher(Int32, '/target_tag', 10)
        self.lift_cmd_pub = self.create_publisher(Float64, '/lift_position', 10)
        
        # Subscribers (optional tag following)
        if self.enable_tag_following:
            self.tag_reached_sub = self.create_subscription(
                Bool, '/tag_reached', self.tag_reached_callback, 10)
        
        # Timer for state machine (10 Hz)
        self.timer = self.create_timer(0.1, self.state_machine)
        
        # Mission sequence from parameters
        self.tag_sequence = {
            self.GO_TO_DOCK1: self.get_parameter('dock1_tags').value,
            self.GO_TO_DOCK2: self.get_parameter('dock2_tags').value,
            self.GO_TO_CHARGER: self.get_parameter('charger_tags').value
        }
        
        self.current_tag_index = 0
        self.state_start_time = None
        self.mission_started = False
        
        self.get_logger().info(f'Mission Controller Started')
        self.get_logger().info(f'  Tag following: {self.enable_tag_following}')
        self.get_logger().info(f'  Background camera: {self.enable_camera}')
        self.get_logger().info(f'  Mission start delay: {self.mission_delay}s')
        
        # Start mission after configured delay
        self.create_timer(self.mission_delay, self.start_mission)

    def start_mission(self):
        """Initialize mission sequence"""
        if self.mission_started:
            return
        self.mission_started = True
        self.state = self.GO_TO_DOCK1
        self.current_tag_index = 0
        self.state_start_time = time.time()
        self.get_logger().info('=== STARTING MISSION ===')
        self.get_logger().info(f'Docking Station 1 sequence: {self.tag_sequence[self.GO_TO_DOCK1]}')

    def tag_reached_callback(self, msg):
        """Callback when tag is reached (only if tag following enabled)"""
        if msg.data:
            self.tag_reached = True
            elapsed = time.time() - self.state_start_time if self.state_start_time else 0
            self.get_logger().debug(f'Tag reached callback (state={self.state}, elapsed={elapsed:.2f}s)')

    def publish_target_tag(self, tag_id):
        """Publish target tag ID if tag following is enabled"""
        if self.enable_tag_following:
            msg = Int32()
            msg.data = tag_id
            self.target_tag_pub.publish(msg)
            self.get_logger().info(f'🎯 Target tag: {tag_id}')
        else:
            self.get_logger().debug(f'Tag following disabled, skipping publish for tag {tag_id}')

    def set_lift_position(self, position):
        """Publish lift command"""
        msg = Float64()
        msg.data = position
        self.lift_cmd_pub.publish(msg)
        state_name = "UP" if position > 0.25 else "DOWN"
        self.get_logger().info(f'🔧 Lift: {state_name} (pos={position:.2f})')

    def check_tag_sequence_complete(self):
        """Check if all tags in current sequence reached"""
        if not self.enable_tag_following:
            # If tag following disabled, use timeout instead
            elapsed = time.time() - self.state_start_time
            return elapsed > self.tag_timeout
        return self.current_tag_index >= len(self.tag_sequence[self.state])

    def advance_tag_sequence(self):
        """Advance to next tag in sequence"""
        if self.current_tag_index < len(self.tag_sequence[self.state]):
            tag_id = self.tag_sequence[self.state][self.current_tag_index]
            self.publish_target_tag(tag_id)
            self.tag_reached = False
            self.current_tag_index += 1
            return True
        return False

    def state_machine(self):
        """Main mission state machine"""
        
        if self.state == self.IDLE:
            # Waiting for mission start
            pass
        
        elif self.state == self.GO_TO_DOCK1:
            # Navigate to Docking Station 1
            if self.state_start_time is None:
                self.state_start_time = time.time()
                self.get_logger().info('📍 [GO_TO_DOCK1] Starting navigation to Docking Station 1')
                self.advance_tag_sequence()
            
            elif (self.enable_tag_following and self.tag_reached) or (not self.enable_tag_following):
                if not self.advance_tag_sequence():
                    # All tags reached
                    self.get_logger().info('✅ [GO_TO_DOCK1] Arrived at Docking Station 1')
                    self.state = self.LIFT_UP
                    self.state_start_time = time.time()
                    self.current_tag_index = 0
            
            # Timeout check
            elif time.time() - self.state_start_time > self.tag_timeout:
                self.get_logger().warn(f'⚠️ [GO_TO_DOCK1] Tag timeout after {self.tag_timeout}s, proceeding to lift')
                self.state = self.LIFT_UP
                self.state_start_time = time.time()
                self.current_tag_index = 0
        
        elif self.state == self.LIFT_UP:
            # Lift the rack up
            if self.state_start_time is None:
                self.state_start_time = time.time()
                self.get_logger().info('⬆️ [LIFT_UP] Lifting rack to upper position')
                self.set_lift_position(0.5)
            
            elif time.time() - self.state_start_time > self.lift_up_time:
                self.get_logger().info('✅ [LIFT_UP] Rack fully lifted')
                self.state = self.GO_TO_DOCK2
                self.state_start_time = time.time()
                self.current_tag_index = 0
        
        elif self.state == self.GO_TO_DOCK2:
            # Navigate to Docking Station 2
            if self.state_start_time is None:
                self.state_start_time = time.time()
                self.get_logger().info('📍 [GO_TO_DOCK2] Starting navigation to Docking Station 2')
                self.advance_tag_sequence()
            
            elif (self.enable_tag_following and self.tag_reached) or (not self.enable_tag_following):
                if not self.advance_tag_sequence():
                    # All tags reached
                    self.get_logger().info('✅ [GO_TO_DOCK2] Arrived at Docking Station 2')
                    self.state = self.LIFT_DOWN
                    self.state_start_time = time.time()
                    self.current_tag_index = 0
            
            # Timeout check
            elif time.time() - self.state_start_time > self.tag_timeout:
                self.get_logger().warn(f'⚠️ [GO_TO_DOCK2] Tag timeout after {self.tag_timeout}s, proceeding to lower')
                self.state = self.LIFT_DOWN
                self.state_start_time = time.time()
                self.current_tag_index = 0
        
        elif self.state == self.LIFT_DOWN:
            # Lower the rack down
            if self.state_start_time is None:
                self.state_start_time = time.time()
                self.get_logger().info('⬇️ [LIFT_DOWN] Lowering rack to lower position')
                self.set_lift_position(0.0)
            
            elif time.time() - self.state_start_time > self.lift_down_time:
                self.get_logger().info('✅ [LIFT_DOWN] Rack fully lowered')
                self.state = self.GO_TO_CHARGER
                self.state_start_time = time.time()
                self.current_tag_index = 0
        
        elif self.state == self.GO_TO_CHARGER:
            # Return to charging station
            if self.state_start_time is None:
                self.state_start_time = time.time()
                self.get_logger().info('📍 [GO_TO_CHARGER] Returning to charging station')
                self.advance_tag_sequence()
            
            elif (self.enable_tag_following and self.tag_reached) or (not self.enable_tag_following):
                if not self.advance_tag_sequence():
                    # Mission complete
                    self.get_logger().info('🎉 [MISSION COMPLETE] Successfully returned to charging station')
                    self.state = self.DONE
                    self.state_start_time = None
            
            # Timeout check
            elif time.time() - self.state_start_time > self.tag_timeout:
                self.get_logger().warn(f'⚠️ [GO_TO_CHARGER] Tag timeout after {self.tag_timeout}s, mission complete')
                self.state = self.DONE
                self.state_start_time = None
        
        elif self.state == self.DONE:
            # Mission finished
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
