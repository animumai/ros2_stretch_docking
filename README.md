Setup headcamera position backwards:
```bash
ros2 topic pub --once /joint_pose_cmd std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.14159, -1,0472, 0.0, 0.0]}"
```

Dock:
```bash
ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot   '{"use_dock_id": true, "dock_id": "home_dock"}'
```

Undock:
```bash
ros2 action send_goal /undock_robot opennav_docking_msgs/action/UndockRobot "{}"
```