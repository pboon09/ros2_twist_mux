#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

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
        self.emergency_active = self.enable_emergency  # Start in safe state if enabled
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
        input_names = self.get_parameter('inputs').get_parameter_value().string_array_value

        for name in input_names:
            self.declare_parameter(f"{name}.topic", f"/cmd_vel_{name}")
            self.declare_parameter(f"{name}.priority", 0)
            self.declare_parameter(f"{name}.timeout", 0.5)

            topic = self.get_parameter(f"{name}.topic").value
            priority = self.get_parameter(f"{name}.priority").value
            timeout = self.get_parameter(f"{name}.timeout").value

            sub = self.create_subscription(
                Twist,
                topic,
                lambda msg, input_name=name: self.input_callback(msg, input_name),
                10
            )

            self.inputs.append({
                'name': name,
                'topic': topic,
                'priority': priority,
                'timeout': timeout,
                'subscriber': sub,
                'last_msg': None,
                'last_time': None
            })

        # Create timer (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Twist Mux Init")

    def emergency_callback(self, msg):
        if self.enable_emergency:
            self.last_emergency_time = self.get_clock().now()
            self.emergency_active = msg.data

    def check_emergency(self):
        if not self.enable_emergency:
            return False
            
        if self.last_emergency_time is None:
            return True  # Emergency active until first message
            
        timeout = self.get_parameter('emergency_timeout').value
        elapsed = (self.get_clock().now() - self.last_emergency_time).nanoseconds * 1e-9
        return self.emergency_active or (elapsed > timeout)

    def input_callback(self, msg, input_name):
        for input_config in self.inputs:
            if input_config['name'] == input_name:
                input_config['last_msg'] = msg
                input_config['last_time'] = self.get_clock().now()
                break

    def timer_callback(self):
        # Emergency check (only when enabled)
        if self.enable_emergency and self.check_emergency():
            self.publish_zero_twist()
            self.get_logger().warn("Emergency stop active!")
            return

        # Normal operation
        current_time = self.get_clock().now()
        active_inputs = []

        for input_config in self.inputs:
            if input_config['last_time'] is not None:
                elapsed = (current_time - input_config['last_time']).nanoseconds * 1e-9
                if elapsed < input_config['timeout']:
                    active_inputs.append(input_config)

        if active_inputs:
            active_inputs.sort(key=lambda x: x['priority'], reverse=True)
            selected = active_inputs[0]
            self.publisher.publish(selected['last_msg'])
            self.get_logger().debug(f"Selected input: {selected['name']}")
        else:
            self.publish_zero_twist()
            self.get_logger().debug("No active inputs")

    def publish_zero_twist(self):
        zero_twist = Twist()
        zero_twist.linear.x = 0.0
        zero_twist.angular.z = 0.0
        self.publisher.publish(zero_twist)

def main(args=None):
    rclpy.init(args=args)
    node = TwistMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()