# ROS2 Stretch Docking

This package provides a docking solution for a Stretch robot (real or simulated) using ROS2, Nav2, and a visual servoing loop to align the robot with a docking station equipped with an ArUco marker.

---

## Installation

Follow the same installation instructions as for the Stretch [Nav2](https://github.com/animumai/stretch_nav2_offloaded) package with the addition of installing the latest version of `opencv-contrib-python`:
```bash
pip install opencv-contrib-python
```


---

## Docking Procedure

1. **Initialization**: The robot pitches its camera down, looks backward, and stows its arm. Nav2 is then used to navigate to a staging position near the docking station.
2. **ArUco Marker Detection**: When the ArUco marker is detected, the arm is lowered to allow continuous visibility of the marker. The visual servoing loop begins to align the robot with the docking station.
3. **Marker Lost**: If the marker is lost, the robot returns to the staging position using Nav2.
4. **Docking Complete**: The docking process stops when the robot is aligned and positioned within predefined tolerances.

---

## Launch Files

### `nav2_docking.launch.py`
This is the primary launch file for executing the docking procedure. It launches:
- **Docking Node**: Executes the docking behavior.
- **ArUco Detection Node**: Publishes the pose of the docking station relative to the robot.

### `aruco_detect.launch.py`
This auxiliary launch file can be used to echo the `/detected_dock_pose` topic. It helps in setting up the `external_detection_offsets` parameter by manually adjusting and observing the detected pose.

---

## Parameters

### `nav2_docking.launch.py` Parameters
- `map_yaml_file`: Path to the map file used by Nav2.
- `dock_pose`: Pose of the docking station in the map frame `[x, y, theta]`.
- `staging_offset`: Offset from the docking station to the staging position `[x_offset, y_offset]`.
- `external_detection_offsets`: Adjustments for the final visual servoing alignment `[x_offset, y_offset, z_offset]`.

### `docking_node.py` Parameters
- `map_yaml_file`: Same as above.
- `dock_pose`: Same as above.
- `staging_offset`: Same as above.
- `external_detection_offsets`: Same as above.

---

## Prerequisites

Before launching any docking-related files, ensure the robot and Nav2 are set up:

### For the Real Stretch Robot
1. **Power Up Stretch**: Ensure it is untethered and connected to the same network as the external computer.
2. **Launch the Stretch Driver and LiDAR**:
   ```bash
   # Free up Stretch processes
   stretch_free_robot_process.py

   # Start the Stretch Camera Node
   ros2 launch stretch_core d435i_basic.launch.py 
   # ros2 launch stretch_core multi_camera_demo.launch.py
   ```
3. **Launch Nav2**:
   ```bash
   ros2 launch stretch_nav2 navigation.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
   ```
4. **Set the 2D Pose Estimate** in Rviz.
5. **Start docking by launching**:
   ``` bash
   ros2 launch ros2_stretch_docking nav2_docking.launch.py
   ```

### For the Simulator
1. **Launch the Simulation**:
   ```bash
   ros2 launch ros2_simulator_stretch ros2_simulator_stretch.launch.py
   ```
2. **Launch Nav2**:
   ```bash
   ros2 launch stretch_nav2_offloaded navigation_sim.launch.py map:=src/ros2_simulator_stretch/maps/<map_name>.yaml
   ```
   Changing `<map_name>` for the name of the map, e.g., `supermarket`.
3. **Set the 2D Pose Estimate** in Rviz. If you don't see the inflation layer appearing afterwards, close and relaunch the navigation.
4. **Start docking by launching**:
   ``` bash
   ros2 launch ros2_stretch_docking nav2_docking_sim.launch.py
   ```

---

**Maintainer**: [Victor Nan Fernandez-Ayala](mailto:victor@animum.ai)  
**License**: Copyright Animum AB 2025
