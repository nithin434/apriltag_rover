#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float64
from std_srvs.srv import Empty
import time

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        self.IDLE = 0
        self.GO_TO_DOCK1 = 1
        self.LIFT_UP = 2
        self.GO_TO_DOCK2 = 3
        self.LIFT_DOWN = 4
        self.GO_TO_CHARGER = 5
        self.DONE = 6
        
        self.state = self.IDLE
        self.tag_reached = False
        
        self.target_tag_pub = self.create_publisher(Int32, '/target_tag', 10)
        self.lift_cmd_pub = self.create_publisher(Float64, '/lift_position', 10)
        
        self.tag_reached_sub = self.create_subscription(
            Bool, '/tag_reached', self.tag_reached_callback, 10)
        
        self.timer = self.create_timer(0.1, self.state_machine)
        
        self.tag_sequence = {
            self.GO_TO_DOCK1: [1, 2],
            self.GO_TO_DOCK2: [3, 4],
            self.GO_TO_CHARGER: [5, 0]
        }
        
        self.current_tag_index = 0
        self.lift_timer = 0
        
        self.get_logger().info('Mission Controller Started')
        self.create_timer(3.0, self.start_mission, True)

    def start_mission(self):
        self.get_logger().info('=== STARTING MISSION ===')
        self.state = self.GO_TO_DOCK1
        self.current_tag_index = 0

    def tag_reached_callback(self, msg):
        if msg.data:
            self.tag_reached = True

    def publish_target_tag(self, tag_id):
        msg = Int32()
        msg.data = tag_id
        self.target_tag_pub.publish(msg)
        self.get_logger().info(f'Target tag set to: {tag_id}')

    def set_lift_position(self, position):
        msg = Float64()
        msg.data = position
        self.lift_cmd_pub.publish(msg)
        self.get_logger().info(f'Lift position: {position}')

    def state_machine(self):
        if self.state == self.IDLE:
            pass
        elif self.state == self.GO_TO_DOCK1:
            if self.current_tag_index == 0:
                self.get_logger().info('[STATE] Going to Docking Station 1')
                self.publish_target_tag(self.tag_sequence[self.state][0])
                self.tag_reached = False
                self.current_tag_index += 1
            elif self.tag_reached and self.current_tag_index < len(self.tag_sequence[self.state]):
                self.publish_target_tag(self.tag_sequence[self.state][self.current_tag_index])
                self.tag_reached = False
                self.current_tag_index += 1
            elif self.tag_reached and self.current_tag_index >= len(self.tag_sequence[self.state]):
                self.get_logger().info('[STATE] Arrived at Docking Station 1')
                self.state = self.LIFT_UP
                self.lift_timer = 0
                self.current_tag_index = 0
        elif self.state == self.LIFT_UP:
            if self.lift_timer == 0:
                self.get_logger().info('[STATE] Lifting rack UP')
                self.set_lift_position(0.5)
                self.lift_timer = time.time()
            elif time.time() - self.lift_timer > 3.0:
                self.get_logger().info('[STATE] Rack lifted!')
                self.state = self.GO_TO_DOCK2
                self.current_tag_index = 0
        elif self.state == self.GO_TO_DOCK2:
            if self.current_tag_index == 0:
                self.get_logger().info('[STATE] Going to Docking Station 2')
                self.publish_target_tag(self.tag_sequence[self.state][0])
                self.tag_reached = False
                self.current_tag_index += 1
            elif self.tag_reached and self.current_tag_index < len(self.tag_sequence[self.state]):
                self.publish_target_tag(self.tag_sequence[self.state][self.current_tag_index])
                self.tag_reached = False
                self.current_tag_index += 1
            elif self.tag_reached and self.current_tag_index >= len(self.tag_sequence[self.state]):
                self.get_logger().info('[STATE] Arrived at Docking Station 2')
                self.state = self.LIFT_DOWN
                self.lift_timer = 0
                self.current_tag_index = 0
        elif self.state == self.LIFT_DOWN:
            if self.lift_timer == 0:
                self.get_logger().info('[STATE] Lowering rack DOWN')
                self.set_lift_position(0.0)
                self.lift_timer = time.time()
            elif time.time() - self.lift_timer > 3.0:
                self.get_logger().info('[STATE] Rack lowered!')
                self.state = self.GO_TO_CHARGER
                self.current_tag_index = 0
        elif self.state == self.GO_TO_CHARGER:
            if self.current_tag_index == 0:
                self.get_logger().info('[STATE] Returning to Charging Station')
                self.publish_target_tag(self.tag_sequence[self.state][0])
                self.tag_reached = False
                self.current_tag_index += 1
            elif self.tag_reached and self.current_tag_index < len(self.tag_sequence[self.state]):
                self.publish_target_tag(self.tag_sequence[self.state][self.current_tag_index])
                self.tag_reached = False
                self.current_tag_index += 1
            elif self.tag_reached and self.current_tag_index >= len(self.tag_sequence[self.state]):
                self.get_logger().info('[STATE] Mission COMPLETE!')
                self.state = self.DONE
        elif self.state == self.DONE:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
