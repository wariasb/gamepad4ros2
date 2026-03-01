# Gamepad to ROS 2

Reads a gamepad from Linux evdev and publishes ROS 2 topics:

- `/control/steering_command` from left analog X (left/right)
- `/control/throttle_command` from right analog Y (up)
- `/control/brake_command` from right analog Y (down)

## Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select gamepad4ros2
```

## Run

```bash
source install/setup.bash
ros2 run gamepad4ros2 gamepad_to_ros2 --ros-args \
  -p device:=/dev/input/event0 \
  -p left_x_code:=0 \
  -p right_y_code:=5 \
  -p deadzone:=0.05
```

The axis codes use Linux input event codes (e.g. `ABS_X`=0, `ABS_RY`=5).
Use `evtest` to identify the correct event device and axis codes if needed.

## Permissions

If you get permission errors, add a udev rule or run with elevated permissions.
