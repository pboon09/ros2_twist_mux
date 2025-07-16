# Twist Mux for ROS2 Humble

A ROS2 node that intelligently multiplexes multiple `geometry_msgs/Twist` velocity commands into a single output topic. The node prioritizes non-zero velocity commands and includes an optional emergency stop mechanism for safety-critical applications.

## Features

### 1. **Smart Input Selection**
- Subscribes to multiple `geometry_msgs/Twist` topics
- Automatically selects the highest-priority input with non-zero velocity
- Falls back to lower-priority inputs when higher-priority inputs publish zero velocities
- Maintains highest priority selection when all inputs are zero

### 2. **Timeout Handling**
- Each input source has a configurable timeout period
- Inputs that haven't published within their timeout are automatically excluded from selection
- Prevents stale commands from being used

### 3. **Emergency Stop System** (Optional)
- Monitors a `std_msgs/Bool` topic for emergency conditions
- Forces zero velocity output when emergency is active
- Includes timeout protection - triggers emergency if no "safe" signal received
- Can be completely disabled via configuration

### 4. **Flexible Configuration**
- All settings configurable via ROS2 parameters
- Support for arbitrary number of input sources
- Per-input priority and timeout settings

## Installation

### Prerequisites
- ROS2 Humble
- Python 3

### Setup
```bash
mkdir -p ~/twist_mux_ws/src
cd ~/twist_mux_ws/src

git clone https://github.com/pboon09/ros2_twist_mux.git

cd ~/twist_mux_ws
colcon build --packages-select ros2_twist_mux

source install/setup.bash
```

## Configuration

Create a YAML configuration file (e.g., `config/twist_mux.yaml`):

```yaml
twist_mux:
  ros__parameters:
    # Output topic for final velocity commands
    output_topic: "cmd_vel"
    
    # Emergency stop configuration
    enable_emergency: true          # Set to false to disable emergency system
    emergency_topic: "emergency_stop"
    emergency_timeout: 0.5          # Seconds before timeout triggers emergency
    
    # List of input source names
    inputs: ["teleop", "nav"]
    
    # Teleop input configuration
    teleop:
      topic: "/cmd_vel_teleop"
      priority: 100                 # Higher number = higher priority
      timeout: 0.5                  # Seconds before input is considered inactive
    
    # Navigation input configuration  
    nav:
      topic: "/cmd_vel_nav"
      priority: 50
      timeout: 0.5
```

### Configuration Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `output_topic` | string | Topic name for multiplexed velocity output |
| `inputs` | string[] | List of input source names to configure |
| `enable_emergency` | bool | Enable/disable emergency stop system |
| `emergency_topic` | string | Topic name for emergency stop signal (std_msgs/Bool) |
| `emergency_timeout` | float | Time in seconds before emergency activates on signal loss |
| `<input>.topic` | string | Topic name for this input source |
| `<input>.priority` | int | Priority level (higher values take precedence) |
| `<input>.timeout` | float | Time in seconds before input is considered inactive |

## Usage

### Example Setup
```bash
ros2 launch ros2_twist_mux ros2_twist_mux.launch.py
```

## Important Notes

### Publishing Rate Consistency
⚠️ **For optimal performance, all input sources should publish at the same rate.** The twist mux operates in an event-driven manner, forwarding messages at the rate of the active input. Mismatched publishing rates between inputs can cause irregular output rates when switching between sources.

### Priority Behavior
- The node always selects the highest-priority input that:
  1. Is within its timeout period
  2. Has non-zero velocity data
- If all active inputs have zero velocity, the highest priority input is used

### Emergency Stop Behavior
When emergency stop is triggered (either by explicit signal or timeout):
- All velocity commands are overridden with zero
- Output continues at the rate of incoming messages
- Normal operation resumes immediately when emergency clears

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No output on cmd_vel | Check that at least one input is publishing within its timeout |
| Unexpected input selected | Verify priority settings and that higher priority inputs aren't publishing zeros |
| Emergency won't clear | Ensure emergency_stop topic is publishing `false` within timeout period |
| Irregular output rate | Ensure all input sources publish at the same rate |

## Feedback
If you have any feedback, please create an issue and I will answer your questions there.