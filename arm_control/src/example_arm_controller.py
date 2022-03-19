#! /usr/bin/env python
import sys
import subprocess
import rospy 
import actionlib
import time
from infrastructure_msgs.msg import StageAction, StageGoal, StageFeedback, StageResult
# for running example paths
from general_path_planner_kinova import MoveRobot

class ExampleArmController():
	
	def __init__(self):   
		#initializing actionservers
		self.start_arm = actionlib.SimpleActionServer("start_arm_sequence", StageAction, self.start_arm_sequence_callback, False) 
		self.start_arm.start()
		# Better method, importing class from general path planner. Change which csv used to change path.
		# Note: csv file must be within joint_angles folder next to general_path_planner_kinova.py
		example_controller = MoveRobot(0, 0, "testbed_example_path.csv")
 
	def start_arm_sequence_callback(self, goal):
	
		self.start_arm.publish_feedback(StageFeedback(status="EXAMPLE: GRABBING OBJECT"))
		###do any arm calls or work here
		
		# For running testbed example script:
		# (quick method of running a process and blocking. Uses general path planner script to read joint angle csv with real kinova robot)
		#subprocess.call(["/home/testbed-tower/kinova_ws/src/kinova-ros/kinova_scripts/src/kinova_infrastructure_planning/kinova_scripts/src/general_path_planner_kinova.py", "0", "0", "/home/testbed-tower/kinova_ws/src/kinova-ros/kinova_scripts/src/kinova_infrastructure_planning/kinova_scripts/src/joint_angles/better_testbed.csv"])
		# better implementation:
		example_controller.Run()

		# For manual stop:
		#user_in = raw_input("press Enter")
		#time.sleep(1)
		
		self.start_arm.set_succeeded(StageResult(result = 0), text="SUCCESS")



if __name__ == '__main__':
	rospy.init_node("example_arm_controller", argv=sys.argv)
	begin = ExampleArmController()
	rospy.spin()
