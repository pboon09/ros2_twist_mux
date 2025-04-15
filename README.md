# Twist Mux for ROS2 Humble
A simple multiplexing node that merges multiple Twist commands into a single velocity output (`cmd_vel`).
Optional: Can integrate an emergency stop mechanism that forces zero velocity if an emergency condition is active or if no “safe” signal has been received.

## Features
1. Multiple Input Sources
  - Subscribes to multiple `geometry_msgs/Twist topics`.
  - Selects the highest-priority active input to publish.
2. Timeout Handling
  -Each input can define a timeout (in seconds).
  -If the node does not receive a Twist message from that input within the timeout period, that input is ignored.
3. Fail-Safe Emergency Stop (Optional)
  - A boolean topic can be used to override all velocity commands.
  - If set to `true` (or if no `false` messages are received within the timeout period), it publishes a zero Twist for safety.
  - **Disable** this feature via parameter if you do not need it.
4. Configurable via Parameters
  - You can specify:
    - Input topic names, priorities, and timeouts
    - Output topic name
    - Emergency topic name
    - Emergency enable/disable and timeout

## Cloning and Setting Up the Package
1. Clone the custom message package 
```
cd ~/ros2_ws/src
git clone https://github.com/pboon09/twist_mux.git
```
2. Build the Package After cloning, build the package with
```
cd ~/ros2_ws
colcon build
```

## Configuration
```yaml
twist_mux:
  ros__parameters:
    # The final published cmd_vel topic
    output_topic: "cmd_vel"

    # Emergency settings
    emergency_topic: "emergency_stop"
    emergency_timeout: 0.5
    enable_emergency: true

    # Which input sources to listen for
    inputs: ["teleop", "nav"]

    # Teleop parameters
    teleop:
      topic: "/cmd_vel_teleop"
      priority: 100
      timeout: 0.5

    # Navigation parameters
    nav:
      topic: "/cmd_vel_nav"
      priority: 50
      timeout: 0.5
```

**Notable Parameters**
- `output_topic`: The final velocity command topic (e.g., /cmd_vel).
- `inputs`: A list of names referencing individual input settings.
- `<input_name>.topic`: The topic to which this input listens for Twist messages.
- `<input_name>.priority`: Higher values override lower ones if both inputs are active.
- `<input_name>.timeout`: Time in seconds after which an input is ignored if no new message arrives.
- `enable_emergency`: Set to true to enable emergency logic, false to disable.
- `emergency_topic`: Name of the std_msgs/Bool topic controlling the emergency stop.
- `emergency_timeout`: Time in seconds after which the emergency is considered active if no false message is received.

## Usage
You can run by a launch file:
```
ros2 launch twist_mux twist_mux.launch.py
```
If you prefer running it directly:
```
ros2 run twist_mux twist_mux_node --ros-args --params-file /path/to/twist_mux.yaml
```
