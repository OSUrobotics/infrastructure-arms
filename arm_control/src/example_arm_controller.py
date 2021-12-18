#! /usr/bin/env python
import sys
import subprocess
import rospy 
import actionlib
import time
from infrastructure_msgs.msg import StageAction, StageGoal, StageFeedback, StageResult

class ExampleArmController():
	
	def __init__(self):   
		#initializing actionservers
		self.start_arm = actionlib.SimpleActionServer("start_arm_sequence", StageAction, self.start_arm_sequence_callback, False) 
		self.start_arm.start()
		
 
	def start_arm_sequence_callback(self, goal):

	
		self.start_arm.publish_feedback(StageFeedback(status="EXAMPLE: GRABBING OBJECT"))
		###do any arm calls or work here
		# can have path planner in same directory as example_arm_controller.py
		#subprocess.call(["/home/sogol/kinova_Ws/src/kinova-ros/kinova_scripts/src/general_path_planner_kinova.py", "0", "0", "/home/sogol/kinova_Ws/src/kinova-ros/kinova_scripts/src/joint_angles/drawer_path.csv"])
		#subprocess.call(["/home/testbed-tower/kinova_ws/src/kinova-ros/kinova_scripts/src/kinova_infrastructure_planning/kinova_scripts/src/general_path_planner_kinova.py", "0", "0", "/home/testbed-tower/kinova_ws/src/kinova-ros/kinova_scripts/src/kinova_infrastructure_planning/kinova_scripts/src/joint_angles/better_testbed.csv"])
		#pseudo timer
		user_in = raw_input("press key")
		time.sleep(1)
		self.start_arm.set_succeeded(StageResult(result = 0), text="SUCCESS")



if __name__ == '__main__':
	rospy.init_node("example_arm_controller", argv=sys.argv)
	begin = ExampleArmController()
	rospy.spin()
