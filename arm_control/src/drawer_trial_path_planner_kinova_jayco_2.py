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
from tf.transformations import *
import tf.transformations as tfs
import numpy as np
import json
import os

class DrawerArmController():
    
    def __init__(self):
        # Joint angles path
        __here__ = os.path.dirname(__file__)
        self.darwer_arm_presets: dict = json.load(f'{__here__}/joint_angles/drawer.json')
        self.drawer_parameters: dict = json.load(f'{__here__}/drawer/drawer_parameters.json')

        #initializing actionservers
        self.start_arm = actionlib.SimpleActionServer("start_arm_sequence", StageAction, self.start_arm_sequence_callback, False) 
        self.start_arm.start()
        #print(real_robot)
        real_robot = True
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
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        


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


        # Add constraint areas 
        self._add_constraint_box(pos=[.63,0,1], dim=[.1,2,2], name="drawer")
        self._add_constraint_box(pos=[0,0,-.05], dim=[2,2,.1], name="table")
        self._add_constraint_box(pos=[-.53,.762,.5], dim=[.1,.25,1], orient=[0.0,0.0,1.0,.5], name="rear_left_camera")
        self._add_constraint_box(pos=[-.53,-.762,.5], dim=[.1,.25,1], orient=[0.0,0.0,1.0,-.5], name="rear_right_camera")
        self._add_constraint_box(pos=[.475,0,1.1], dim=[.2,2,.1], orient=[0.0,1.0,0.0,-.1], name="overhead_light")
        self._add_constraint_box(pos=[.46,-0.09,.28], dim=[0.07, 0.25, 0.07], name="handle")
        # knob: radius: 3.004, height: 2 + 3/4 + 0.10
        self._add_constraint_cylinder(self.drawer_parameters["knob"])
        rospy.loginfo("Added scene constraints.")

        # # Start pose
        # self.start_pose = list(map(lambda k: self.darwer_arm_presets[k], sorted(self.darwer_arm_presets["initial_joint_position"].keys())))
        self.start_pose = [
            self.darwer_arm_presets["initial_joint_position"]["joint1"],
            self.darwer_arm_presets["initial_joint_position"]["joint2"],
            self.darwer_arm_presets["initial_joint_position"]["joint3"],
            self.darwer_arm_presets["initial_joint_position"]["joint4"],
            self.darwer_arm_presets["initial_joint_position"]["joint5"],
            self.darwer_arm_presets["initial_joint_position"]["joint6"],
            self.darwer_arm_presets["initial_joint_position"]["joint7"],
        ]


        rospy.sleep(3)
        self.joint_angle_rounded = 2 

        # Move to start position          
        self.run_arm_to_start_pose()

        # Go to standard pull position
        self.current_pose = self.move_group.get_current_pose()
        self.target_pose = self.generate_pose([.37, .3, .31], [90, 0, 50])
        self.run_arm_to_target_pose()

        rospy.signal_shutdown("done")
        hey = raw_input("arm at target pose")



        # self.current_pose = self.move_group.get_current_pose()
        # print("Current pose: ", self.current_pose)
        # self.current_pose.pose.position.y += .1
        # self.move_group.set_pose_target(self.current_pose)
        # out = self.move_group.go(wait=True)
        # self.move_group.stop()
        # self.move_group.clear_pose_targets()
        # self.current_pose = self.move_group.get_current_pose()
        # print("Current pose: ", self.current_pose)


        # rospy.signal_shutdown("done")
        # hey = raw_input("Enter to move to cartesian pull position")


        # self.current_pose.pose.position.x -= 0
        # self.current_pose.pose.position.y += -.1
        # self.current_pose.pose.position.z += .2
        # self.current_joint_values = self.move_group.get_current_joint_values() # How to get current joint positions
        # print("Joint angles", self.current_joint_values)
        # self.move_group.set_pose_target(self.current_pose)
        # out = self.move_group.go(wait=True)
        # self.move_group.stop()
        # self.move_group.clear_pose_targets()

        # self.current_joint_values = self.move_group.get_current_joint_values() # How to get current joint positions
        # self.current_pose = self.move_group.get_current_pose() #
        # print("Joint angles", self.current_joint_values)
        # print("Target pose", self.current_pose)
        # #user = raw_input("Waiting to move on")

        # self.current_pose.pose.position.x += .1
        # self.current_pose.pose.position.y -= .15
        # self.current_pose.pose.position.z += 0
        # self.move_group.set_pose_target(self.current_pose)
        # out = self.move_group.go(wait=True)
        # self.move_group.stop()
        # self.move_group.clear_pose_targets()


    def run_arm_to_joint_orientation(self, joint_angles: list) -> None:
        """ Run the arm to a specific joint orientation """
        self.move_group.set_joint_value_target(joint_angles)
        self.move_group.plan()
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        self.current_joint_values = self.move_group.get_current_joint_values() # How to get current joint positions
        print("Joint angles", self.current_joint_values)
        return
    

    def run_arm_to_start_pose(self) -> None:
        """ Run the arm to the starting pose """
        return self.run_arm_to_joint_orientation(self.start_pose)


    def run_arm_to_target_pose(self) -> None:
        """ Run the end effector to a specific position and orientation """
        self.move_group.set_pose_target(self.target_pose)
        out = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        print(f"Reached target: {self.target_pose}")
        return
    
        """
        self.current_pose = self.move_group.get_current_pose() # How to get current pose
        q_down = quaternion_from_euler(0,0,-.5)
        current_q = [self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w]
        q_new = quaternion_multiply(q_down, current_q)

        self.current_pose.pose.orientation.x = q_new[0]
        self.current_pose.pose.orientation.y = q_new[1]
        self.current_pose.pose.orientation.z = q_new[2]
        self.current_pose.pose.orientation.w = q_new[3]
        self.move_group.set_pose_target(self.current_pose)
        out = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        self.current_pose.pose.position.x += 0
        self.current_pose.pose.position.y += 0
        self.current_pose.pose.position.z -= .03
        self.move_group.set_pose_target(self.current_pose)
        out = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        self.move_group.set_joint_value_target([11.331234850300119, 2.478875668967935, 2.573784604947852, 0.6257295386642381, 4.493317349114771, 1.1381729982464674, 9.406469260907302])
        out = self.move_group.go(wait=True)
        self.move_group.stop()
        
        self.current_pose.pose.orientation.x = 0
        self.current_pose.pose.orientation.y = 0
        self.current_pose.pose.orientation.z = -.5
        self.current_pose.pose.orientation.w = 0

        
        q_down = quaternion_from_euler(0,0,0)
        current_q = [self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w]
        q_new = quaternion_multiply(q_down, current_q)

        self.current_pose.pose.orientation.x = q_new[0]
        self.current_pose.pose.orientation.y = q_new[1]
        self.current_pose.pose.orientation.z = q_new[2]
        self.current_pose.pose.orientation.w = q_new[3]
        



        self.move_group.set_pose_target(self.current_pose)
        out = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        """


        self.current_pose = self.move_group.get_current_pose() # How to get current pose
        print("Updated pose", self.current_pose)
        #print(Pose())


 
    def start_arm_sequence_callback(self, goal):
    
        self.start_arm.publish_feedback(StageFeedback(status="EXAMPLE: GRABBING OBJECT"))
        #user_n = raw_input("bro")
        # Do arm call here
        rospy.sleep(1.0)
        self.start_arm.set_succeeded(StageResult(result = 0), text="SUCCESS")

    def modify_arm_pose_cartesian(self, ):
        # 
        print("bro")

    def generate_pose(self, position: list, orientation: list) -> np.ndarray:
        """ Generate a pose from a position and orientation 
        @param position: position of the end effector in meters
        @param orientation: orientation of the end effector in degrees
        """
        quat = tfs.quaternion_from_euler(math.radians(orientation[0]),math.radians(orientation[1]),math.radians(orientation[2]))
        temp_pose = Pose()
        temp_pose.position.x = position[0]
        temp_pose.position.y = position[1]
        temp_pose.position.z = position[2]
        temp_pose.orientation.x = quat[0]
        temp_pose.orientation.y = quat[1]
        temp_pose.orientation.z = quat[2]
        temp_pose.orientation.w = quat[3]

        return temp_pose


    def _add_constraint_box(self, name, dim, pos, orient=[0.0,0.0,1.0,0.0]) -> bool:
        """ Add a constraint box to the scene """
        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        
        box_pose.pose.position.x = pos[0] 
        box_pose.pose.position.y = pos[1]  
        box_pose.pose.position.z = pos[2] 
        box_pose.pose.orientation.x = orient[0]
        box_pose.pose.orientation.y = orient[1]
        box_pose.pose.orientation.z = orient[2]
        box_pose.pose.orientation.w = orient[3]
        box_name = name
        self.scene.add_box(box_name, box_pose, size=(dim[0], dim[1], dim[2]))

        if self._check_object_presence(box_name):
            return True
        else:
            return False
    
    def _add_constraint_cylinder(self, cyl: dict) -> bool:
        """ Add a constraint cylinder to the scene """
        cyl_pose = PoseStamped()
        cyl_pose.header.frame_id = "world"
        cyl_name = cyl["name"]
        cyl_height = cyl["dimensions"]["height"]
        cyl_radius = cyl["dimensions"]["radius"]

        cyl_pose.pose.position.x = cyl["position"]["x"]
        cyl_pose.pose.position.y = cyl["position"]["y"]
        cyl_pose.pose.position.z = cyl["position"]["z"]

        quat = tfs.quaternion_from_euler(cyl["orientation"]["x"], cyl["orientation"]["y"], cyl["orientation"]["z"])
        
        cyl_pose.pose.orientation.x = quat[0]
        cyl_pose.pose.orientation.y = quat[1]
        cyl_pose.pose.orientation.z = quat[2]
        cyl_pose.pose.orientation.w = quat[3]
        
        self.scene.add_cylinder(cyl_name, cyl_pose, cyl_height, cyl_radius)

        if self._check_object_presence(cyl_name):
            return True
        else:
            return False


    def _check_object_presence(self, obj_name: str) -> bool:
        """ Check to see if the constraint object was successfully added """
        start_time = time_now = rospy.get_time()
        while (time_now - start_time < 5) and not rospy.is_shutdown():
            # Test if the object is in the scene
            # NOTE: attaching teh box will remove it from known_objects
            is_known = obj_name in self.scene.get_known_object_names()
            # See if we are in the expected state
            if is_known:
                print(f"Added constraint {obj_name}")
                return True
            
            # sleep, giving other threads time on the processor
            rospy.sleep(0.1)
            time_now = rospy.get_time()
        # If we exited teh loop, then we timed out
        print(f"Failed adding constraint {obj_name}")
        return False
    

    def run(self):

        return




if __name__ == '__main__':
    rospy.init_node("drawer_arm_controller_what", argv=sys.argv)
    begin = DrawerArmController()
    rospy.spin()
