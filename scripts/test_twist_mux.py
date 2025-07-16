#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class TestTwistMux(Node):
    def __init__(self):
        super().__init__('test_twist_mux')
        
        self.joystick_pub = self.create_publisher(Twist, '/cmd_vel/joystick', 10)
        self.teleop_pub = self.create_publisher(Twist, '/cmd_vel/teleop', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.emergency_stop_state = False
        self.last_button_state = False
        
        self.latest_joy_msg = None
        
        self.timer = self.create_timer(0.01, self.timer_callback)
                
        self.get_logger().info("test_twist_mux node started - publishing at 100Hz")
        
    def joy_callback(self, msg):
        self.latest_joy_msg = msg
        
        if len(msg.buttons) > 0:
            current_button_state = msg.buttons[0] == 1
            if current_button_state and not self.last_button_state:
                self.emergency_stop_state = not self.emergency_stop_state
            self.last_button_state = current_button_state
    
    def timer_callback(self):
        if self.latest_joy_msg is None:
            joystick_twist = Twist()
            teleop_twist = Twist()
        else:
            joystick_twist = Twist()
            joystick_twist.linear.x = self.latest_joy_msg.axes[1]
            joystick_twist.linear.y = self.latest_joy_msg.axes[0]
            
            teleop_twist = Twist()
            teleop_twist.linear.x = self.latest_joy_msg.axes[4]
            teleop_twist.linear.y = self.latest_joy_msg.axes[3]
        
        self.joystick_pub.publish(joystick_twist)
        self.teleop_pub.publish(teleop_twist)
        
        emergency_msg = Bool()
        emergency_msg.data = self.emergency_stop_state
        self.emergency_stop_pub.publish(emergency_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestTwistMux()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()