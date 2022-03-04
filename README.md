# infrastructure-arms
## Overview
### arm_control package:
Used to control the Kinova Jaco2 with the infrastructure system. Currently used as a stop while the script for controlling the Kinova gen2 is ran seperately.

### kinova_\* packages:
Modified source code and configuration packages for the Kinova Jaco2. Modifications include:
- Increased joint velocity to 2.0 in [_joint_limits.yaml_](https://github.com/OSUrobotics/infrastructure-arms/blob/Kinova_j2s7s300/kinova_moveit/robot_configs/j2s7s300_moveit_config/config/joint_limits.yaml).
- Added files for data visualization capabilities in RVIZ

### RVIZ Data Visualization files:
- [__drawer_j2s7s300_virtual.launch__](https://github.com/OSUrobotics/infrastructure-arms/blob/Kinova_j2s7s300/kinova_moveit/robot_configs/j2s7s300_moveit_config/launch/drawer_j2s7s300_virtual.launch)
  - Main launch file for launching entire Data Visualization system for the Drawer.
  - Launches all launch files related to Data visualization.
  - To run:
    ```console
    roslaunch j2s7s300_moveit_config drawer_j2s7s300_virtual.launch data:=<string> position:='x y z' rotation:='r p y'
    ```
  - Launch parameters:
    ```
    data:=<string> (path name for CSV file containing Drawer trial information)

    position:='x y z' (Position of Drawer model (in meters) in RVIZ, relative to the origin)
  
    rotation:='r p y' (Rotation of Drawer model (in radians) in RVIZ, relative to the origin)
    ```
- [__j2s7s300_virtual_robot_demo_visualization.launch__](https://github.com/OSUrobotics/infrastructure-arms/blob/Kinova_j2s7s300/kinova_moveit/robot_configs/j2s7s300_moveit_config/launch/j2s7s300_virtual_robot_demo_visualization.launch)
  - Launches a Kinova Jaco2 robot model that works with multiple robot models in RVIZ
  - Similar to j2s7s300_virtual_robot_demo.launch except for these changes:
    - Spawns a static transform publisher node that broadcasts the static tf for the robot root
    - Doesn't launch RVIZ
  
- [__drawer_spawn.launch__](https://github.com/OSUrobotics/infrastructure-arms/blob/Kinova_j2s7s300/kinova_description/launch/drawer_spawn.launch)
  - Launches a Drawer robot model that works with multiple robot models in RVIZ
    - Note: looks for the [urdf files](https://github.com/OSUrobotics/infrastructure-raspi/tree/drawer/infrastructure_raspi/urdf) for the Drawer model inside the infrastructure_raspi package.
  - Launches [drawer_updater](https://github.com/OSUrobotics/infrastructure-arms/blob/Kinova_j2s7s300/kinova_description/src/drawer_updater.py) node that publishes to the _/drawer_distance_ topic which controls the joint pose of the Drawer robot model. Currently, drawer_updater reads from the csv file passed into the drawer_j2s7s300_virtual.launch file.
  - __For testing:__ launches [data_intermediary](https://github.com/OSUrobotics/infrastructure-arms/blob/Kinova_j2s7s300/kinova_description/src/data_intermediary.py)  and [data_plotter](https://github.com/OSUrobotics/infrastructure-arms/blob/Kinova_j2s7s300/kinova_description/src/data_plotter.py) nodes. data_intermediary subscribes to _/drawer_distance_ and provides a service for data_plotter which creates and updates a PyQt plot for the drawer distance in real time.

- [__drawer_moveit_rviz.launch__](https://github.com/OSUrobotics/infrastructure-arms/blob/Kinova_j2s7s300/kinova_moveit/robot_configs/j2s7s300_moveit_config/launch/drawer_moveit_rviz.launch)
  - Launches RVIZ window that is setup for viewing the Kinova and Drawer robot model.
    - Uses the [drawer_moveit.rviz](https://github.com/OSUrobotics/infrastructure-arms/blob/Kinova_j2s7s300/kinova_moveit/robot_configs/j2s7s300_moveit_config/launch/drawer_moveit.rviz) configuration file.

## arm_control Package Interface
### Action Servers:
- __start_arm_sequence__
  - Action server that the _User Arm Control_ stage action client sends a goal to.
  - Used to signal start of arm control. Sends result once arm control has finished
### Services:
- None
### Publishers:
- None
### Subscribers:
- None
### Topics:
- /start_arm_sequence/cancel
- /start_arm_sequence/feedback
- /start_arm_sequence/goal
- /start_arm_sequence/result
- /start_arm_sequence/status
