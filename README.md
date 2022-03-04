# infrastructure-arms
## Overview
Contains ROS package (arm_control) for arm control with the infrastructure system. Each branch contains setup and control for different arms. The _main_ branch contains a template for the ROS package (see [example_arm_controller.py](https://github.com/OSUrobotics/infrastructure-arms/blob/main/arm_control/src/example_arm_controller.py)).

## infrastructure_arms Package Interface
### Action Servers:
- start_arm_sequence
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
