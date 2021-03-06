# infrastructure-arms Template
## Overview
Contains ROS package (arm_control) for arm control with the infrastructure system. Each branch contains setup and control for different arms. The _main_ branch contains a template for the ROS package (see [example_arm_controller.py](https://github.com/OSUrobotics/infrastructure-arms/blob/main/arm_control/src/example_arm_controller.py)).

__Note:__ With current implementation, you must not change the file name of [example_arm_controller.py](https://github.com/OSUrobotics/infrastructure-arms/blob/main/arm_control/src/example_arm_controller.py) (see [infrastructure_flexbe_behaviors](https://github.com/OSUrobotics/infrastructure-packages/tree/new_file_structure/infrastructure_behaviors#infrastructure_flexbe_behaviors-package-overview)).

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
