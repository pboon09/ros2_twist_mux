#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from functools import partial

class ROS2TwistMuxNode(Node):
    def __init__(self):
        super().__init__('ros2_twist_mux')

        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('inputs', ['teleop', 'nav'])
        self.declare_parameter('emergency_topic', 'emergency_stop')
        self.declare_parameter('emergency_timeout', 0.5)
        self.declare_parameter('enable_emergency', True)

        self.enable_emergency = self.get_parameter('enable_emergency').value
        self.emergency_active = False
        self.last_emergency_time = None

        if self.enable_emergency:
            emergency_topic = self.get_parameter('emergency_topic').value
            self.emergency_sub = self.create_subscription(
                Bool,
                emergency_topic,
                self.emergency_callback,
                10
            )
            self.emergency_timer = self.create_timer(0.1, self.emergency_timeout_check)
            self.get_logger().info(f"Emergency system enabled on topic: {emergency_topic}")
        else:
            self.get_logger().info("Emergency system disabled")

        self.publisher = self.create_publisher(
            Twist, 
            self.get_parameter('output_topic').value,
            10
        )

        self.inputs = []
        input_names = self.get_parameter('inputs').value

        for i, name in enumerate(input_names):
            self.declare_parameter(f"{name}.topic", f"/cmd_vel_{name}")
            self.declare_parameter(f"{name}.priority", 0)
            self.declare_parameter(f"{name}.timeout", 0.5)

            topic = self.get_parameter(f"{name}.topic").value
            priority = self.get_parameter(f"{name}.priority").value
            timeout = self.get_parameter(f"{name}.timeout").value

            callback = partial(self.input_callback, input_name=name)
            
            sub = self.create_subscription(
                Twist,
                topic,
                callback,
                10
            )

            self.inputs.append({
                'name': name,
                'topic': topic,
                'priority': priority,
                'timeout': timeout,
                'subscriber': sub,
                'last_msg': None,
                'last_time': None,
                'index': i
            })

        self.inputs.sort(key=lambda x: (-x['priority'], x['index']))

        self.current_active_input = None
        self.last_published_was_zero = False
        
        self.get_logger().info(f"Twist Mux initialized with inputs: {[inp['name'] for inp in self.inputs]}")
        self.get_logger().info(f"Priority order: {[(inp['name'], inp['priority']) for inp in self.inputs]}")

    def emergency_callback(self, msg):
        if self.enable_emergency:
            self.last_emergency_time = self.get_clock().now()
            prev_state = self.emergency_active
            self.emergency_active = msg.data
            
            if prev_state != self.emergency_active:
                if self.emergency_active:
                    self.get_logger().warn("Emergency stop activated!")
                else:
                    self.get_logger().info("Emergency stop deactivated")

    def emergency_timeout_check(self):
        if not self.enable_emergency:
            return
            
        if self.last_emergency_time is None:
            if not self.emergency_active:
                self.emergency_active = True
                # self.get_logger().warn("No emergency data received - activating emergency stop")
            return
            
        timeout = self.get_parameter('emergency_timeout').value
        elapsed = (self.get_clock().now() - self.last_emergency_time).nanoseconds * 1e-9
        
        if elapsed > timeout and not self.emergency_active:
            self.emergency_active = True
            # self.get_logger().warn("Emergency topic timeout - activating emergency stop")

    def check_emergency(self):
        if not self.enable_emergency:
            return False
            
        if self.last_emergency_time is None:
            return True
            
        timeout = self.get_parameter('emergency_timeout').value
        elapsed = (self.get_clock().now() - self.last_emergency_time).nanoseconds * 1e-9
        
        return self.emergency_active or (elapsed > timeout)

    def is_twist_zero(self, msg):
        if msg is None:
            return True
        
        return (msg.linear.x == 0.0 and 
                msg.linear.y == 0.0 and 
                msg.linear.z == 0.0 and
                msg.angular.x == 0.0 and 
                msg.angular.y == 0.0 and 
                msg.angular.z == 0.0)

    def get_active_input(self):
        current_time = self.get_clock().now()
        
        active_inputs = []
        for input_config in self.inputs:
            if input_config['last_time'] is not None:
                elapsed = (current_time - input_config['last_time']).nanoseconds * 1e-9
                if elapsed < input_config['timeout']:
                    active_inputs.append(input_config)
        
        if not active_inputs:
            return None
        
        for input_config in active_inputs:
            if not self.is_twist_zero(input_config['last_msg']):
                return input_config
        
        return active_inputs[0]

    def input_callback(self, msg, input_name):
        input_config = None
        for config in self.inputs:
            if config['name'] == input_name:
                config['last_msg'] = msg
                config['last_time'] = self.get_clock().now()
                input_config = config
                break
        
        if input_config is None:
            # self.get_logger().error(f"Unknown input source: {input_name}")
            return

        if self.check_emergency():
            self.publish_zero_twist()
            self.current_active_input = None
            return

        active_input = self.get_active_input()
        
        if active_input is None:
            self.publish_zero_twist()
            self.current_active_input = None
            return
        
        if active_input['name'] == input_name:
            self.publisher.publish(msg)
            self.current_active_input = input_name
            self.last_published_was_zero = self.is_twist_zero(msg)
            
            # if not self.is_twist_zero(msg):
            #     self.get_logger().debug(f"Published non-zero from input: {input_name} (priority: {active_input['priority']})")
        
        elif self.current_active_input != active_input['name']:
            prev_input = self.current_active_input
            self.current_active_input = active_input['name']
            if active_input['last_msg'] is not None:
                self.publisher.publish(active_input['last_msg'])
                self.last_published_was_zero = self.is_twist_zero(active_input['last_msg'])
                
                # if prev_input and not self.is_twist_zero(active_input['last_msg']):
                #     self.get_logger().info(f"Switched from {prev_input} to {active_input['name']} (priority: {active_input['priority']}) - has non-zero data")
                # else:
                #     self.get_logger().info(f"Switched to input: {active_input['name']} (priority: {active_input['priority']})")

    def publish_zero_twist(self):
        zero_twist = Twist()
        zero_twist.linear.x = 0.0
        zero_twist.linear.y = 0.0
        zero_twist.linear.z = 0.0
        zero_twist.angular.x = 0.0
        zero_twist.angular.y = 0.0
        zero_twist.angular.z = 0.0
        
        self.publisher.publish(zero_twist)
        self.last_published_was_zero = True
        # self.get_logger().debug("Published zero twist")

def main(args=None):
    rclpy.init(args=args)
    node = ROS2TwistMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TwistMux node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()