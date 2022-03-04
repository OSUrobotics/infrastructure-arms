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
    roslaunch j2s7s300_moveit_config drawer_j2s7s300_virtual.launch
    ```
  - Launch parameters:
    ```
    data:=<string> (path name for CSV file containing Drawer trial information)

    position:='x y z' (Position of Drawer model in RVIZ, relative to the origin)
  
    rotation:='r p y' (Rotation of Drawer model in RVIZ, relative to the origin)
    ```

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
