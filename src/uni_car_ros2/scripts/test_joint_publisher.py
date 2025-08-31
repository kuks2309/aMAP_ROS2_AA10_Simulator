#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class TestJointPublisher(Node):
    def __init__(self):
        super().__init__('test_joint_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.counter = 0
        
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Joint names for the uni_car including steer joints and rear_wheel
        msg.name = [
            'front_steer_joint',
            'front_left_steer_joint', 
            'front_right_steer_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        
        # Rotate wheels and steer
        wheel_angle = self.counter * 0.1
        steer_angle = math.sin(self.counter * 0.02) * 0.3  # Slow oscillating steering
        
        msg.position = [
            steer_angle,   # front_steer_joint
            steer_angle,   # front_left_steer_joint
            steer_angle,   # front_right_steer_joint
            wheel_angle,   # front_left_wheel
            wheel_angle,   # front_right_wheel
            wheel_angle,   # rear_wheel (virtual center wheel)
            wheel_angle,   # rear_left_wheel
            wheel_angle    # rear_right_wheel
        ]
        
        msg.velocity = [0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5]
        msg.effort = []
        
        self.publisher.publish(msg)
        self.counter += 1
        
        if self.counter % 10 == 0:
            self.get_logger().info(f'Publishing wheel angle: {wheel_angle:.2f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = TestJointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()