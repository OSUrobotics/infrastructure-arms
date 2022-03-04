# infrastructure-arms
## Overview
### arm_control:
Used to control the Kinova gen2 with the infrastructure system. Currently used as a stop while the script for controlling the Kinova gen2 is ran seperately.

### kinova_\*:
Modified packages


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
