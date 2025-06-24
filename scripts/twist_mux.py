#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from functools import partial

class TwistMuxNode(Node):
    def __init__(self):
        super().__init__('twist_mux')

        # Declare parameters
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('inputs', ['teleop', 'nav'])
        self.declare_parameter('emergency_topic', 'emergency_stop')
        self.declare_parameter('emergency_timeout', 0.5)
        self.declare_parameter('enable_emergency', True)

        # Initialize emergency system state
        self.enable_emergency = self.get_parameter('enable_emergency').value
        self.emergency_active = False  # Start in normal state
        self.last_emergency_time = None

        # Setup emergency system only if enabled
        if self.enable_emergency:
            emergency_topic = self.get_parameter('emergency_topic').value
            self.emergency_sub = self.create_subscription(
                Bool,
                emergency_topic,
                self.emergency_callback,
                10
            )
            # Create emergency timeout timer (10Hz for checking timeout)
            self.emergency_timer = self.create_timer(0.1, self.emergency_timeout_check)
            self.get_logger().info(f"Emergency system enabled on topic: {emergency_topic}")
        else:
            self.get_logger().info("Emergency system disabled")

        # Create output publisher
        self.publisher = self.create_publisher(
            Twist, 
            self.get_parameter('output_topic').value,
            10
        )

        # Initialize input handlers
        self.inputs = []
        input_names = self.get_parameter('inputs').value

        for i, name in enumerate(input_names):
            self.declare_parameter(f"{name}.topic", f"/cmd_vel_{name}")
            self.declare_parameter(f"{name}.priority", 0)
            self.declare_parameter(f"{name}.timeout", 0.5)

            topic = self.get_parameter(f"{name}.topic").value
            priority = self.get_parameter(f"{name}.priority").value
            timeout = self.get_parameter(f"{name}.timeout").value

            # FIX: Use partial to properly bind the input_name
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
                'index': i  # Add index for stable sorting
            })

        # Sort inputs by priority (highest first), then by index for stability
        self.inputs.sort(key=lambda x: (-x['priority'], x['index']))

        # Track current active input and state
        self.current_active_input = None
        self.last_published_was_zero = False
        
        self.get_logger().info(f"Twist Mux initialized with inputs: {[inp['name'] for inp in self.inputs]}")
        self.get_logger().info(f"Priority order: {[(inp['name'], inp['priority']) for inp in self.inputs]}")

    def emergency_callback(self, msg):
        """Handle emergency stop messages"""
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
        """Check for emergency timeout (runs at 10Hz)"""
        if not self.enable_emergency:
            return
            
        # FIX: Handle startup case - if no emergency data received, this IS emergency
        if self.last_emergency_time is None:
            if not self.emergency_active:
                self.emergency_active = True
                self.get_logger().warn("No emergency data received - activating emergency stop")
            return
            
        timeout = self.get_parameter('emergency_timeout').value
        elapsed = (self.get_clock().now() - self.last_emergency_time).nanoseconds * 1e-9
        
        # If timeout occurred and we weren't already in emergency
        if elapsed > timeout and not self.emergency_active:
            self.emergency_active = True
            self.get_logger().warn("Emergency topic timeout - activating emergency stop")

    def check_emergency(self):
        """Check if emergency stop should be active"""
        if not self.enable_emergency:
            return False
            
        # FIX: If we haven't received any emergency messages, this IS emergency state
        if self.last_emergency_time is None:
            return True  # No emergency data = emergency active
            
        # Check for timeout
        timeout = self.get_parameter('emergency_timeout').value
        elapsed = (self.get_clock().now() - self.last_emergency_time).nanoseconds * 1e-9
        
        # Emergency is active if explicitly set OR if timeout occurred
        return self.emergency_active or (elapsed > timeout)

    def get_active_input(self):
        """Find the highest priority active input"""
        current_time = self.get_clock().now()
        
        # Check inputs in priority order (already sorted)
        for input_config in self.inputs:
            if input_config['last_time'] is not None:
                elapsed = (current_time - input_config['last_time']).nanoseconds * 1e-9
                if elapsed < input_config['timeout']:
                    return input_config
        return None

    def input_callback(self, msg, input_name):
        """Handle incoming twist messages - publish at input rate"""
        # Find the correct input configuration
        input_config = None
        for config in self.inputs:
            if config['name'] == input_name:
                config['last_msg'] = msg
                config['last_time'] = self.get_clock().now()
                input_config = config
                break
        
        if input_config is None:
            self.get_logger().error(f"Unknown input source: {input_name}")
            return

        # Check emergency first
        if self.check_emergency():
            # Emergency active: publish zero at same rate as input
            self.publish_zero_twist()
            self.current_active_input = None
            return

        # Find current highest priority active input
        active_input = self.get_active_input()
        
        if active_input is None:
            # No active inputs - publish zero at rate of this input
            self.publish_zero_twist()
            self.current_active_input = None
            return
        
        # If this input is the currently active one, publish its data
        if active_input['name'] == input_name:
            self.publisher.publish(msg)
            self.current_active_input = input_name
            self.last_published_was_zero = False
            self.get_logger().debug(f"Published from input: {input_name} (priority: {active_input['priority']})")
        
        # If active input changed to a different one, switch immediately
        elif self.current_active_input != active_input['name']:
            self.current_active_input = active_input['name']
            if active_input['last_msg'] is not None:
                self.publisher.publish(active_input['last_msg'])
                self.last_published_was_zero = False
                self.get_logger().info(f"Switched to input: {active_input['name']} (priority: {active_input['priority']})")
        
        # If this input is not the active one, we still received data but don't publish
        # The active input will publish at its own rate

    def publish_zero_twist(self):
        """Publish a zero velocity command"""
        zero_twist = Twist()
        zero_twist.linear.x = 0.0
        zero_twist.linear.y = 0.0
        zero_twist.linear.z = 0.0
        zero_twist.angular.x = 0.0
        zero_twist.angular.y = 0.0
        zero_twist.angular.z = 0.0
        
        self.publisher.publish(zero_twist)
        self.last_published_was_zero = True
        self.get_logger().debug("Published zero twist")

def main(args=None):
    rclpy.init(args=args)
    node = TwistMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TwistMux node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()