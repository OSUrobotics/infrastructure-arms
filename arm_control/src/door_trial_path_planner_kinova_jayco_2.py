#!/usr/bin/env python

# Author: Kyle DuFrene
# Email: dufrenek@oregonstate.edu
# Date: 6/16
#
# 

import sys
import subprocess
import rospy 
import actionlib
import time
from infrastructure_msgs.msg import StageAction, StageGoal, StageFeedback, StageResult
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Pose
import moveit_commander
from time import sleep

class DoorArmController():
    
    def __init__(self):   
        #initializing actionservers
        self.start_arm = actionlib.SimpleActionServer("start_arm_sequence", StageAction, self.start_arm_sequence_callback, False) 
        self.start_arm.start()
        #print(real_robot)
        real_robot = False
        # To read from redirected ROS Topic (Gazebo launch use)
        if real_robot:
            joint_state_topic = ['joint_states:=/j2s7s300_driver/out/joint_state']
            moveit_commander.roscpp_initialize(joint_state_topic)
            #rospy.init_node('move_kinova', anonymous=False)
            moveit_commander.roscpp_initialize(sys.argv)
        else:
            joint_state_topic = ['joint_states:=/j2s7s300/joint_states']
            moveit_commander.roscpp_initialize(joint_state_topic)
            #rospy.init_node('move_kinova', anonymous=False)
            moveit_commander.roscpp_initialize(sys.argv)

        # Define robot using RobotCommander. Provided robot info such as
        # kinematic model and current joint state
        self.robot = moveit_commander.RobotCommander()

        # Setting the world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Define the planning group for the arm you are using
        # You can easily look it up on rviz under the MotionPlanning tab
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper")

        rospy.wait_for_service("/apply_planning_scene", 10.0)
        rospy.wait_for_service("/get_planning_scene", 10.0)

        self.apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        self.get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        #rospy.sleep(2)

        # To see the trajectory
        self.disp = DisplayTrajectory()

        self.disp.trajectory_start = self.robot.get_current_state()

        self.rate = rospy.Rate(10)

        self.move_group.allow_replanning(1)

        #how much to round joint angles
        self.joint_angle_rounded = 2 

        # Helpful functions http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
        self.current_joint_values = self.move_group.get_current_joint_values() # How to get current joint positions
        self.current_pose = self.move_group.get_current_pose() # How to get current pose
        self.current_pose.pose.position.x += -.2

        self.move_group.set_pose_target(self.current_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        print(self.current_joint_values)
        print("Current pose", self.current_pose)
        print(Pose())

 
    def start_arm_sequence_callback(self, goal):
    
        self.start_arm.publish_feedback(StageFeedback(status="EXAMPLE: GRABBING OBJECT"))
        #user_n = raw_input("bro")
        # Do arm call here
        sleep(1)
        self.start_arm.set_succeeded(StageResult(result = 0), text="SUCCESS")



if __name__ == '__main__':
    rospy.init_node("door_arm_controller_what", argv=sys.argv)
    begin = DoorArmController()
    rospy.spin()
