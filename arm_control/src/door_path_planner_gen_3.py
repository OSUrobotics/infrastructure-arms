#!/usr/bin/env python
"""
door_path_planner_gen_3.py
Author: Kyle DuFrene
Email: dufrenek@oregonstate.edu
Date: 09/04/2023
"""

import sys
import subprocess
import rospy
import actionlib
import time
from infrastructure_msgs.msg import StageAction, StageGoal, StageFeedback, StageResult, ArmTrialMetadata
import std_msgs.msg
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Pose
import moveit_commander
from time import sleep
import tf.transformations as tfs
import numpy as np
import json
import os
import pprint
from copy import deepcopy
from tf.transformations import *
import math
#rosrun arm_control door_path_planner_gen_3.py __ns:=my_gen3




class DoorArmController:
    def __init__(self, grasp_obj_type):
        self.grasp_obj_type = grasp_obj_type.strip().lower()
        # Joint scene JSON info
        __here__ = os.path.dirname(__file__)
        # Base environment objects
        door_env_base_path = os.path.join(__here__, "door", "door_env_base.json")
        with open(door_env_base_path, "r") as jfile:
            self.door_env_constraints = json.load(jfile)
        # Load handle/knob type object
        grasp_obj_path = os.path.join(__here__, "door", self.grasp_obj_type+".json")
        with open(grasp_obj_path, "r") as jfile:
            grasp_obj = json.load(jfile)
            # Add handle/knob to constraints
            self.door_env_constraints[grasp_obj[self.grasp_obj_type]["name"]] = grasp_obj[self.grasp_obj_type]

        # Load preset poses
        arm_poses_path = os.path.join(__here__,"joint_angles", "door.json") 
        with open(arm_poses_path, "r") as jfile:
            self.arm_poses = json.load(jfile)

        # Initialize Metadata publisher, reset metadata subscriber
        self.metadata_pub = rospy.Publisher("/arm_trial_metadata", ArmTrialMetadata, queue_size=10)
        self.reset_metadata_sub = rospy.Subscriber('/reset_metadata', std_msgs.msg.Float32, self.force_callback)
        
        self.trial_force = 0.0

        # initializing actionservers
        self.start_arm = actionlib.SimpleActionServer(
            "start_arm_sequence", StageAction, self.start_arm_sequence_callback, False)
        self.start_arm.start()
        # print(real_robot)
        
        moveit_commander.roscpp_initialize(sys.argv)
        try:
            print("namespace")
            print(rospy.get_namespace())
            namespace = "/my_gen3/"
            
            #self.is_gripper_present = rospy.get_param("my_gen3" + "is_gripper_present", False)
            self.is_gripper_present = True
            if self.is_gripper_present:
                print("here1")
                self.gripper_joint_name = "finger_joint"
                print("here")
            else:
                gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(namespace + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface(ns=namespace)
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=namespace)
            self.display_trajectory_publisher = rospy.Publisher(namespace + 'move_group/display_planned_path', DisplayTrajectory, queue_size=20)
            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=namespace)
            print("here3")
            print("made it here")

            self.arm_group.set_num_planning_attempts(3)
            self.rate = rospy.Rate(10)
            self.arm_group.allow_replanning(1)

            # Add constraint areas
            self._add_constraints(self.door_env_constraints)
            rospy.loginfo("Added scene constraints.")
            
            self.generation_pose = Pose()
            self.generation_pose.position.x = .57
            self.generation_pose.position.y = 0.0
            self.generation_pose.position.z = .43
            self.generation_pose.orientation.x = .5
            self.generation_pose.orientation.y = .5
            self.generation_pose.orientation.z = .5
            self.generation_pose.orientation.w = .5

            # Start pose
            self.start_pose = list(map(
                lambda k: self.arm_poses["initial_joint_position"][k],
                sorted(self.arm_poses["initial_joint_position"].keys())
            ))
            # Start pose
            self.start_pose_handle = list(map(
                lambda k: self.arm_poses["initial_joint_position_handle"][k],
                sorted(self.arm_poses["initial_joint_position_handle"].keys())
            ))
            self.start_pose_handle_2 = list(map(
                lambda k: self.arm_poses["initial_joint_position_handle_2"][k],
                sorted(self.arm_poses["initial_joint_position_handle_2"].keys())
            ))
            self.start_pose_handle_3 = list(map(
                lambda k: self.arm_poses["initial_joint_position_handle_3"][k],
                sorted(self.arm_poses["initial_joint_position_handle_3"].keys())
            ))
            self.start_pose_handle_4 = list(map(
                lambda k: self.arm_poses["initial_joint_position_handle_4"][k],
                sorted(self.arm_poses["initial_joint_position_handle_4"].keys())
            ))
            self.safe_pose = list(map(
                lambda k: self.arm_poses["safe_home"][k],
                sorted(self.arm_poses["safe_home"].keys())
            ))
            
            self.top_start = list(map(
                lambda k: self.arm_poses["top_start"][k],
                sorted(self.arm_poses["top_start"].keys())
            ))
            print("wtf")
            
            self.grasp_pose = self.generate_pose(
                position=self.arm_poses["grasp_pose_1"]["position"],
                orientation=self.arm_poses["grasp_pose_1"]["orientation"]
            )
            self.grasp_pose_2 = self.generate_pose(
                position=self.arm_poses["grasp_pose_2"]["position"],
                orientation=self.arm_poses["grasp_pose_2"]["orientation"]
            )
            self.grasp_pose_3 = self.generate_pose(
                position=self.arm_poses["grasp_pose_3"]["position"],
                orientation=self.arm_poses["grasp_pose_3"]["orientation"]
            )
            self.grasp_pose_4 = self.generate_pose(
                position=self.arm_poses["grasp_pose_4"]["position"],
                orientation=self.arm_poses["grasp_pose_4"]["orientation"]
            )
            self.grasp_pose_5 = self.generate_pose(
                position=self.arm_poses["grasp_pose_5"]["position"],
                orientation=self.arm_poses["grasp_pose_5"]["orientation"]
            )
            
            self.grasp_pose_1_handle = self.generate_pose(
                position=self.arm_poses["grasp_pose_1_handle"]["position"],
                orientation=self.arm_poses["grasp_pose_1_handle"]["orientation"]
            )
            
            self.grasp_pose_2_handle = self.generate_pose(
                position=self.arm_poses["grasp_pose_2_handle"]["position"],
                orientation=self.arm_poses["grasp_pose_2_handle"]["orientation"]
            )
            
            self.grasp_pose_3_handle = self.generate_pose(
                position=self.arm_poses["grasp_pose_3_handle"]["position"],
                orientation=self.arm_poses["grasp_pose_3_handle"]["orientation"]
            )
            self.grasp_pose_4_handle = self.generate_pose(
                position=self.arm_poses["grasp_pose_4_handle"]["position"],
                orientation=self.arm_poses["grasp_pose_4_handle"]["orientation"]
            )
            self.grasp_pose_5_handle = self.generate_pose(
                position=self.arm_poses["grasp_pose_5_handle"]["position"],
                orientation=self.arm_poses["grasp_pose_5_handle"]["orientation"]
            )
            

            
            
            
            
            
            # self.start_pose = self.generate_pose(
            #     position = self.arm_poses["start_pose"]["position"],
            #     orientation = self.arm_poses["start_pose"]["orientation"]
            # )
            
            self.target_pose = self.start_pose
            self.pose_list = []
            self.move_dist_from_pull_release = 0.1

            
            self.joint_angle_rounded = 2
        except Exception as e:
            print("Failure!!!")
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True   
            
        pose = self.arm_group.get_current_pose("tool_frame")
        print(pose)
        angles = self.arm_group.get_current_joint_values()
        print(angles)
        
        # Try going to start pose
        # self.run_arm_to_joint_orientation(self.top_start)
        # #inp = raw_input("enter")
        # #self.run_arm_to_cartesian_pose(self.grasp_pose)
        # #self.run_arm_to_joint_orientation(self.safe_pose)
        # #print(self.grasp_pose)
        # self.run_arm_to_cartesian_pose(self.grasp_pose_3)
        
        # pose_list = self.gen_circle_poses(self.grasp_pose_3, width = .213, handle_out=.067)
        # self.scene.remove_world_object(name=self.grasp_obj_type)
        # self.scene.remove_world_object(name="door")
        # self.scene.remove_world_object(name="left_curtain")
        # self.reach_gripper_position(1)
        # #inp = raw_input("enter")
        
        # self.run_waypoints(pose_list)
        # rospy.sleep(1)
        # self.reach_gripper_position(0)
        # self.run_arm_to_joint_orientation(self.safe_pose)
        # self.straight_on_grasp_and_pull_2()
        # self.vertical_grasp_and_pull()
        # self.straight_on_grasp_and_pull_3()
        #self.run_arm_to_joint_orientation(self.start_pose_handle)
        #self.run_arm_to_target_pose(self.grasp_pose_3_handle)
        # self.handle_5()
        
        # 1 is out
        # 2 is in 
        # 3 is side 
        
        """ knob pose
        pose: 
  position: 
    x: 0.601378616716
    y: -0.0621195141493
    z: 0.340827749487
  orientation: 
    x: 0.500508909719
    y: 0.498794766712
    z: 0.499671359958
    w: 0.501022099343
    
    -0.6929657009151962, 1.329408604012605, -1.463292503071668, -2.4904106185973953, -2.915493472835374, -1.7544037070926652, -2.968902103232477
    """
        return 
    def handle_5(self):
        self._add_constraints(self.door_env_constraints)
        self.scene.remove_world_object(name="left_curtain")
        #self._add_constraint_box(box={"name":"front_temp","dimensions":{"x":.1,"y":2.0,"z":2.0},"position":{"x":.6,"y":0.0,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        #self._add_constraint_box(box={"name":"side_temp","dimensions":{"x":2.0,"y":.1,"z":2.0},"position":{"x":0.0,"y":.7,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self.reach_gripper_position(0)
        # inp = raw_input("enter")
        self.run_arm_to_joint_orientation(self.start_pose_handle_4)
        # inp = raw_input("enter")
        #self.scene.remove_world_object("front_temp")
        #self.scene.remove_world_object("side_temp")
        #self._add_constraints(self.door_env_constraints)
        
        #self.run_arm_to_joint_orientation(self.start_pose)
        
        #self.run_arm_to_cartesian_pose(self.grasp_pose)
        #self.run_arm_to_joint_orientation(self.safe_pose)
        #print(self.grasp_pose)
        self.run_arm_to_target_pose(self.grasp_pose_5_handle) #[-1.0102797469014826, 0.8313158542803766, -2.0282589588304942, -2.250081635506464, -0.640769874078515, 0.7996882197536226, 1.225106033077706]
        # sleep(8)
        # angles = self.arm_group.get_current_joint_values()
        # print(angles)
        # sys.exit()
        
        pose_list = self.gen_circle_poses(self.grasp_pose_5_handle, width = .21, handle_out=.103)
        self.scene.remove_world_object(name=self.grasp_obj_type)
        self.scene.remove_world_object(name="door")
        self.scene.remove_world_object(name="left_curtain")
        self.reach_gripper_position(1)
        #inp = raw_input("enter")
        
        self.run_waypoints(pose_list)
        #rospy.sleep(1)
        self.reach_gripper_position(0)
        
    def handle_4(self):
        self._add_constraints(self.door_env_constraints)
        self.scene.remove_world_object(name="left_curtain")
        #self._add_constraint_box(box={"name":"front_temp","dimensions":{"x":.1,"y":2.0,"z":2.0},"position":{"x":.6,"y":0.0,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        #self._add_constraint_box(box={"name":"side_temp","dimensions":{"x":2.0,"y":.1,"z":2.0},"position":{"x":0.0,"y":.7,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self.reach_gripper_position(0)
        # inp = raw_input("enter")
        self.run_arm_to_joint_orientation(self.start_pose_handle_3)
        # inp = raw_input("enter")
        #self.scene.remove_world_object("front_temp")
        #self.scene.remove_world_object("side_temp")
        #self._add_constraints(self.door_env_constraints)
        
        #self.run_arm_to_joint_orientation(self.start_pose)
        
        #self.run_arm_to_cartesian_pose(self.grasp_pose)
        #self.run_arm_to_joint_orientation(self.safe_pose)
        #print(self.grasp_pose)
        self.run_arm_to_target_pose(self.grasp_pose_4_handle) #[-1.0102797469014826, 0.8313158542803766, -2.0282589588304942, -2.250081635506464, -0.640769874078515, 0.7996882197536226, 1.225106033077706]
        
        
        
        pose_list = self.gen_circle_poses(self.grasp_pose_4_handle, width = .21, handle_out=.1)
        self.scene.remove_world_object(name=self.grasp_obj_type)
        self.scene.remove_world_object(name="door")
        self.scene.remove_world_object(name="left_curtain")
        self.reach_gripper_position(1)
        #inp = raw_input("enter")
        
        self.run_waypoints(pose_list)
        #rospy.sleep(1)
        self.reach_gripper_position(0)
    
    def handle_3(self):
        self._add_constraints(self.door_env_constraints)
        self.scene.remove_world_object(name="left_curtain")
        #self._add_constraint_box(box={"name":"front_temp","dimensions":{"x":.1,"y":2.0,"z":2.0},"position":{"x":.6,"y":0.0,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        #self._add_constraint_box(box={"name":"side_temp","dimensions":{"x":2.0,"y":.1,"z":2.0},"position":{"x":0.0,"y":.7,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self.reach_gripper_position(0)
        # inp = raw_input("enter")
        self.run_arm_to_joint_orientation(self.start_pose_handle)
        # inp = raw_input("enter")
        #self.scene.remove_world_object("front_temp")
        #self.scene.remove_world_object("side_temp")
        #self._add_constraints(self.door_env_constraints)
        
        #self.run_arm_to_joint_orientation(self.start_pose)
        
        #self.run_arm_to_cartesian_pose(self.grasp_pose)
        #self.run_arm_to_joint_orientation(self.safe_pose)
        #print(self.grasp_pose)
        self.run_arm_to_cartesian_pose(self.grasp_pose_3_handle)
        
        
        
        pose_list = self.gen_circle_poses(self.grasp_pose_3_handle, width = .20, handle_out=.12)
        self.scene.remove_world_object(name=self.grasp_obj_type)
        self.scene.remove_world_object(name="door")
        self.scene.remove_world_object(name="left_curtain")
        self.reach_gripper_position(1)
        #inp = raw_input("enter")
        
        self.run_waypoints(pose_list)
        #rospy.sleep(1)
        self.reach_gripper_position(0)
    
    def handle_2(self):
        self._add_constraints(self.door_env_constraints)
        self.scene.remove_world_object(name="left_curtain")
        #self._add_constraint_box(box={"name":"front_temp","dimensions":{"x":.1,"y":2.0,"z":2.0},"position":{"x":.6,"y":0.0,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        #self._add_constraint_box(box={"name":"side_temp","dimensions":{"x":2.0,"y":.1,"z":2.0},"position":{"x":0.0,"y":.7,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self.reach_gripper_position(0)
        # inp = raw_input("enter")
        self.run_arm_to_joint_orientation(self.start_pose_handle)
        # inp = raw_input("enter")
        #self.scene.remove_world_object("front_temp")
        #self.scene.remove_world_object("side_temp")
        self._add_constraints(self.door_env_constraints)
        
        #self.run_arm_to_joint_orientation(self.start_pose)
        
        #self.run_arm_to_cartesian_pose(self.grasp_pose)
        #self.run_arm_to_joint_orientation(self.safe_pose)
        #print(self.grasp_pose)
        self.run_arm_to_cartesian_pose(self.grasp_pose_2_handle)
        
        
        pose_list = self.gen_circle_poses(self.grasp_pose_2_handle, width = .21, handle_out=.103)
        self.scene.remove_world_object(name=self.grasp_obj_type)
        self.scene.remove_world_object(name="door")
        self.scene.remove_world_object(name="left_curtain")
        self.reach_gripper_position(1)
        #inp = raw_input("enter")
        
        self.run_waypoints(pose_list)
        #rospy.sleep(1)
        self.reach_gripper_position(0)
        
    def handle_1(self):
        self._add_constraints(self.door_env_constraints)
        self.scene.remove_world_object(name="left_curtain")
        #self._add_constraint_box(box={"name":"front_temp","dimensions":{"x":.1,"y":2.0,"z":2.0},"position":{"x":.6,"y":0.0,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        #self._add_constraint_box(box={"name":"side_temp","dimensions":{"x":2.0,"y":.1,"z":2.0},"position":{"x":0.0,"y":.7,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self.reach_gripper_position(0)
        # inp = raw_input("enter")
        self.run_arm_to_joint_orientation(self.start_pose_handle)
        # inp = raw_input("enter")
        #self.scene.remove_world_object("front_temp")
        #self.scene.remove_world_object("side_temp")
        self._add_constraints(self.door_env_constraints)
        
        #self.run_arm_to_joint_orientation(self.start_pose)
        
        #self.run_arm_to_cartesian_pose(self.grasp_pose)
        #self.run_arm_to_joint_orientation(self.safe_pose)
        #print(self.grasp_pose)
        self.run_arm_to_target_pose(self.grasp_pose_1_handle)
        
        pose_list = self.gen_circle_poses(self.grasp_pose_1_handle, width = .21, handle_out=.140)
        self.scene.remove_world_object(name=self.grasp_obj_type)
        self.scene.remove_world_object(name="door")
        self.scene.remove_world_object(name="left_curtain")
        self.reach_gripper_position(1)
        #inp = raw_input("enter")
        
        self.run_waypoints(pose_list)
        #rospy.sleep(1)
        self.reach_gripper_position(0)
    
    def straight_on_grasp_and_pull_3(self):
        self._add_constraint_box(box={"name":"front_temp","dimensions":{"x":.1,"y":2.0,"z":2.0},"position":{"x":.62,"y":0.0,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self._add_constraint_box(box={"name":"side_temp","dimensions":{"x":2.0,"y":.1,"z":2.0},"position":{"x":0.0,"y":.7,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self.reach_gripper_position(0)
        self.run_arm_to_joint_orientation(self.safe_pose)
        self.scene.remove_world_object("front_temp")
        self.scene.remove_world_object("side_temp")
        self._add_constraints(self.door_env_constraints)
        
        #self.run_arm_to_joint_orientation(self.start_pose)
        
        #self.run_arm_to_cartesian_pose(self.grasp_pose)
        #self.run_arm_to_joint_orientation(self.safe_pose)
        #print(self.grasp_pose)
        self.run_arm_to_target_pose(self.grasp_pose_5)
        
        pose_list = self.gen_circle_poses(self.grasp_pose_5, width = .208, handle_out=.085)
        self.scene.remove_world_object(name=self.grasp_obj_type)
        self.scene.remove_world_object(name="door")
        self.scene.remove_world_object(name="left_curtain")
        self.reach_gripper_position(1)
        #inp = raw_input("enter")
        
        self.run_waypoints(pose_list)
        #rospy.sleep(1)
        self.reach_gripper_position(0)
    
    def vertical_grasp_and_pull(self):
        self._add_constraint_box(box={"name":"front_temp","dimensions":{"x":.1,"y":2.0,"z":2.0},"position":{"x":.62,"y":0.0,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self._add_constraint_box(box={"name":"side_temp","dimensions":{"x":2.0,"y":.1,"z":2.0},"position":{"x":0.0,"y":.7,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self.reach_gripper_position(0)
        self.run_arm_to_joint_orientation(self.safe_pose)
        self.scene.remove_world_object("front_temp")
        self.scene.remove_world_object("side_temp")
        self._add_constraints(self.door_env_constraints)
        
        self.run_arm_to_joint_orientation(self.start_pose)
        self.run_arm_to_cartesian_pose(self.grasp_pose_4)
        pose_list = self.gen_circle_poses(self.grasp_pose_4, width = .208, handle_out=.085)
        self.scene.remove_world_object(name=self.grasp_obj_type)
        self.scene.remove_world_object(name="door")
        self.scene.remove_world_object(name="left_curtain")
        self.reach_gripper_position(1)
        #inp = raw_input("enter")
        
        self.run_waypoints(pose_list)
        self.reach_gripper_position(0)
        
    
    def top_down_grasp_and_pull(self):
        self._add_constraint_box(box={"name":"front_temp","dimensions":{"x":.1,"y":2.0,"z":2.0},"position":{"x":.62,"y":0.0,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self._add_constraint_box(box={"name":"side_temp","dimensions":{"x":2.0,"y":.1,"z":2.0},"position":{"x":0.0,"y":.58,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self.reach_gripper_position(0)
        self.run_arm_to_joint_orientation(self.safe_pose)
        self.scene.remove_world_object("front_temp")
        self.scene.remove_world_object("side_temp")
        self._add_constraints(self.door_env_constraints)
        
        self.run_arm_to_joint_orientation(self.top_start)
        
        #self.run_arm_to_cartesian_pose(self.grasp_pose)
        #self.run_arm_to_joint_orientation(self.safe_pose)
        #print(self.grasp_pose)
        self.run_arm_to_cartesian_pose(self.grasp_pose_3)
        
        pose_list = self.gen_circle_poses(self.grasp_pose_3, width = .213, handle_out=.067)
        self.scene.remove_world_object(name=self.grasp_obj_type)
        self.scene.remove_world_object(name="door")
        self.scene.remove_world_object(name="left_curtain")
        self.reach_gripper_position(1)
        #inp = raw_input("enter")
        
        self.run_waypoints(pose_list)
        #rospy.sleep(1)
        self.reach_gripper_position(0)
    
    def straight_on_grasp_and_pull(self):
        
        self._add_constraint_box(box={"name":"front_temp","dimensions":{"x":.1,"y":2.0,"z":2.0},"position":{"x":.62,"y":0.0,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self._add_constraint_box(box={"name":"side_temp","dimensions":{"x":2.0,"y":.1,"z":2.0},"position":{"x":0.0,"y":.58,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self.reach_gripper_position(0)
        self.run_arm_to_joint_orientation(self.safe_pose)
        self.scene.remove_world_object("front_temp")
        self.scene.remove_world_object("side_temp")
        self._add_constraints(self.door_env_constraints)
        
        self.run_arm_to_joint_orientation(self.start_pose)
        
        #self.run_arm_to_cartesian_pose(self.grasp_pose)
        #self.run_arm_to_joint_orientation(self.safe_pose)
        #print(self.grasp_pose)
        self.run_arm_to_cartesian_pose(self.grasp_pose)
        
        pose_list = self.gen_circle_poses(self.grasp_pose)
        self.scene.remove_world_object(name=self.grasp_obj_type)
        self.scene.remove_world_object(name="door")
        self.scene.remove_world_object(name="left_curtain")
        self.reach_gripper_position(1)
        #inp = raw_input("enter")
        
        self.run_waypoints(pose_list)
        #rospy.sleep(1)
        self.reach_gripper_position(0)
        
    def straight_on_grasp_and_pull_2(self):
        self._add_constraint_box(box={"name":"front_temp","dimensions":{"x":.1,"y":2.0,"z":2.0},"position":{"x":.62,"y":0.0,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self._add_constraint_box(box={"name":"side_temp","dimensions":{"x":2.0,"y":.1,"z":2.0},"position":{"x":0.0,"y":.64,"z":1.0},"orientation":{"x":0.0,"y":0.0,"z":0.0}})
        self.reach_gripper_position(0)
        self.run_arm_to_joint_orientation(self.safe_pose)
        self.scene.remove_world_object("front_temp")
        self.scene.remove_world_object("side_temp")
        self._add_constraints(self.door_env_constraints)
        
        self.run_arm_to_joint_orientation(self.start_pose)
        
        #self.run_arm_to_cartesian_pose(self.grasp_pose)
        #self.run_arm_to_joint_orientation(self.safe_pose)
        #print(self.grasp_pose)
        self.run_arm_to_cartesian_pose(self.grasp_pose_2)
        
        pose_list = self.gen_circle_poses(self.grasp_pose_2, handle_out=.091)
        self.scene.remove_world_object(name=self.grasp_obj_type)
        self.scene.remove_world_object(name="door")
        self.scene.remove_world_object(name="left_curtain")
        self.reach_gripper_position(1)
        #inp = raw_input("enter")
        
        self.run_waypoints(pose_list)
        #rospy.sleep(1)
        self.reach_gripper_position(0)
    
    def reach_gripper_position(self, relative_position):
        #gripper_group = self.gripper_group
        
        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False 
    
    def gen_circle_poses(self, start_gasp_pose, width = .2, handle_out = .05):
        # Build a pose array of arm moving in 1/2 circle from initial grasp
        
        # Start pose 
        # width to handle, distance to center of grasp
        number_poses = 8
        angle_to_open = math.pi/2.5# -.32 # in radians
        angle_per = angle_to_open / (number_poses-1)
        door_width = math.hypot(width, handle_out) #.221
        initial_angle = math.atan(handle_out/width) #.32 # radians
        x_init = math.cos(initial_angle)*door_width
        y_init = math.sin(initial_angle)*door_width
        
        pose_list = [deepcopy(start_gasp_pose)]
        for i in range(number_poses-1):
            start_pose = deepcopy(start_gasp_pose)
            x = math.cos(angle_per*(i+1)+initial_angle)*door_width
            y = math.sin(angle_per*(i+1)+initial_angle)*door_width
            
            
            # dist_apart = 2*door_width*math.sin((angle_per*(i+1))/2)
            # print("Dist ", dist_apart)
            
            # delta_x = math.cos(angle_per*(i+1)/2)*dist_apart*2
            start_pose.position.x -= (y - y_init)
            # delta_y = math.sin(angle_per*(i+1)/2)*dist_apart*2
            start_pose.position.y += (x_init-x)
            
            q_angle = quaternion_from_euler(0, 0, -angle_per*(i+1)) 
            current_q = [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w]
            q_new = quaternion_multiply(q_angle, current_q)
            start_pose.orientation.x = q_new[0]
            start_pose.orientation.y = q_new[1]
            start_pose.orientation.z = q_new[2]
            start_pose.orientation.w = q_new[3]
            
            
            
            
            pose_list.append(deepcopy(start_pose))
        return pose_list
            
            
        
        
   

    def start_arm_sequence_callback(self, goal):
        # Start arm publisher
        self.start_arm.publish_feedback(StageFeedback(status="GRABBING OBJECT"))
        
        #rospy.sleep(1.0)
        
        
        try:
            # Create pose stamped msg
            pose_msg = PoseStamped()
            pose_msg.pose = self.grasp_pose_5_handle
            # Publish metadata
            metadata_msg = ArmTrialMetadata()
            metadata_msg.arm = "Kinova Gen 3"
            metadata_msg.gripper = "RobotIQ 2F-85"
            metadata_msg.grasp_pose = pose_msg
            metadata_msg.pull_type = "normal"
            print(self.trial_force)
            metadata_msg.rotation_or_force = float(self.trial_force)
            
            
            self.handle_5()
            self.metadata_pub.publish(metadata_msg)
            self.start_arm.set_succeeded(StageResult(result=0), text="SUCCESS") 
            
        except IndexError:
            rospy.logerr("Trial sequence ended, no more poses to run.")
        return


    def force_callback(self, force):
        rospy.loginfo("Recieved force input")
        self.trial_force = force.data
        return


    def modify_arm_pose_cartesian(self):
        #
        print("bro")
        return


    def _add_constraints(self, scene_objs):
        for obj in scene_objs.values():
            obj_type = obj["type"]
            if obj_type == "box":
                self._add_constraint_box(obj)
            elif obj_type == "cylinder":
                self._add_constraint_cylinder(obj)
    
    

    def _add_constraint_box(self, box):
        """Add a constraint box to the scene"""
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_name = box["name"]
        box_size = (
            box["dimensions"]["x"],
            box["dimensions"]["y"],
            box["dimensions"]["z"],
        )

        box_pose.pose.position.x = box["position"]["x"]
        box_pose.pose.position.y = box["position"]["y"]
        box_pose.pose.position.z = box["position"]["z"]
        # Get quaternion from 3D orientation
        orients = list(map(
            lambda k: np.radians(box["orientation"][k]),
            sorted(box["orientation"].keys())
        ))
        quat = tfs.quaternion_from_euler(
            orients[0],
            orients[1],
            orients[2]
        )
        box_pose.pose.orientation.x = quat[0]
        box_pose.pose.orientation.y = quat[1]
        box_pose.pose.orientation.z = quat[2]
        box_pose.pose.orientation.w = quat[3]
        
        rospy.loginfo("Adding " + box_name)
        
        self.scene.add_box(box_name, box_pose, size=box_size)
        rospy.sleep(.1)

        if self._check_object_presence(box_name):
            return True
        else:
            return False


    def _add_constraint_cylinder(self, cyl):
        """Add a constraint cylinder to the scene"""
        cyl_pose = PoseStamped()
        cyl_pose.header.frame_id = "base_link"
        cyl_name = cyl["name"]
        cyl_height = cyl["dimensions"]["height"]
        cyl_radius = cyl["dimensions"]["radius"]

        cyl_pose.pose.position.x = cyl["position"]["x"]
        cyl_pose.pose.position.y = cyl["position"]["y"]
        cyl_pose.pose.position.z = cyl["position"]["z"]

        # Get quaternion from 3D orientation
        orients = list(map(
            lambda k: np.radians(cyl["orientation"][k]),
            sorted(cyl["orientation"].keys())
        ))
        quat = tfs.quaternion_from_euler(
            orients[0],
            orients[1],
            orients[2]
        )
        cyl_pose.pose.orientation.x = quat[0]
        cyl_pose.pose.orientation.y = quat[1]
        cyl_pose.pose.orientation.z = quat[2]
        cyl_pose.pose.orientation.w = quat[3]
        rospy.loginfo("Adding " + cyl_name)
        self.scene.add_cylinder(cyl_name, cyl_pose, cyl_height, cyl_radius)
        rospy.sleep(.1)

        if self._check_object_presence(cyl_name):
            return True
        else:
            return False


    def _check_object_presence(self, obj_name):
        """Check to see if the constraint object was successfully added"""
        start_time = time_now = rospy.get_time()
        while (time_now - start_time < 2) and not rospy.is_shutdown():
            # Test if the object is in the scene
            # NOTE: attaching teh box will remove it from known_objects
            is_known = obj_name in self.scene.get_known_object_names()
            # See if we are in the expected state
            if is_known:
                print "Added constraint", obj_name 
                return True

            # sleep, giving other threads time on the processor
            rospy.sleep(0.1)
            time_now = rospy.get_time()
        # If we exited teh loop, then we timed out
        print "Failed adding constraint", obj_name
        return False


    def generate_pose(self, position, orientation):
        """Generate a pose from a position and orientation
        @param position: position of the end effector in meters
        @param orientation: orientation of the end effector in degrees
        """
        print("Generating pose")
        temp_pose = Pose()
        current_q = [self.generation_pose.orientation.x, self.generation_pose.orientation.y, self.generation_pose.orientation.z, self.generation_pose.orientation.w]
        try:
            temp_pose.position.x = position[0]
            temp_pose.position.y = position[1]
            temp_pose.position.z = position[2]
            quat = quaternion_multiply(tfs.quaternion_from_euler(
                np.radians(orientation[0]),
                np.radians(orientation[1]),
                np.radians(orientation[2])
            ), current_q)
        except KeyError:
            temp_pose.position.x = position["x"]
            temp_pose.position.y = position["y"]
            temp_pose.position.z = position["z"]
            quat = quaternion_multiply(tfs.quaternion_from_euler(
                np.radians(orientation["x"]),
                np.radians(orientation["y"]),
                np.radians(orientation["z"])
            ), current_q)
        temp_pose.orientation.x = quat[0]
        temp_pose.orientation.y = quat[1]
        temp_pose.orientation.z = quat[2]
        temp_pose.orientation.w = quat[3]

        return temp_pose
    

    def generate_poses_XTRANS(self, step_size=0.01, num=10, direction=1):
        grasp_pose = self.generate_pose(
            position=self.arm_poses["grasp_pose"]["position"],
            orientation=self.arm_poses["grasp_pose"]["orientation"]
        )
        pose_list = np.empty(10, dtype=Pose)

        for i in range(num):
            modified_pose = deepcopy(grasp_pose)
            modified_pose.position.x += step_size * i * direction
            pose_list[i] = modified_pose
        return pose_list


    def generate_poses_YTRANS(self, step_size=0.01, num=10, direction=1):
        grasp_pose = self.generate_pose(
            position=self.arm_poses["grasp_pose"]["position"],
            orientation=self.arm_poses["grasp_pose"]["orientation"]
        )

        pose_list = np.empty(10, dtype=Pose)

        for i in range(num):
            modified_pose = deepcopy(grasp_pose)
            modified_pose.position.y += step_size * i * direction
            pose_list[i] = modified_pose
        return pose_list
    

    def generate_poses_ZTRANS(self, step_size=0.01, num=10, direction=1):
        grasp_pose = self.generate_pose(
            position=self.arm_poses["grasp_pose"]["position"],
            orientation=self.arm_poses["grasp_pose"]["orientation"]
        )

        pose_list = np.empty(10, dtype=Pose)

        for i in range(num):
            modified_pose = deepcopy(grasp_pose)
            modified_pose.position.z += step_size * i * direction
            pose_list[i] = modified_pose
        return pose_list
    

    def generate_poses_XROT(self, step_size=0.05, num=10, direction=1):
        pose_list = np.empty(10, dtype=Pose)
        for i in range(num):
            modified_pose = deepcopy(self.arm_poses["grasp_pose"])
            modified_pose["orientation"]["x"] += np.degrees(step_size * i * direction)
            pose_list[i] = self.generate_pose(
                position=modified_pose["position"],
                orientation=modified_pose["orientation"]
            )
        return pose_list


    def generate_poses_YROT(self, step_size=0.05, num=10, direction=1):
        pose_list = np.empty(10, dtype=Pose)
        for i in range(num):
            modified_pose = deepcopy(self.arm_poses["grasp_pose"])
            modified_pose["orientation"]["y"] += np.degrees(step_size * i * direction)
            pose_list[i] = self.generate_pose(
                position=modified_pose["position"],
                orientation=modified_pose["orientation"]
            )
        return pose_list
    

    def generate_poses_ZROT(self, step_size=0.05, num=10, direction=1):
        pose_list = np.empty(10, dtype=Pose)
        for i in range(num):
            modified_pose = deepcopy(self.arm_poses["grasp_pose"])
            modified_pose["orientation"]["z"] += np.degrees(step_size * i * direction)
            pose_list[i] = self.generate_pose(
                position=modified_pose["position"],
                orientation=modified_pose["orientation"]
            )
        return pose_list


    def generate_trajectory_away_from_grasp_obj(self):
        curr_pose = self.arm_group.get_current_pose()
        print(curr_pose)
        orientation = list(tfs.euler_from_quaternion([
            curr_pose.pose.orientation.x,
            curr_pose.pose.orientation.y,
            curr_pose.pose.orientation.z,
            curr_pose.pose.orientation.w
        ]))
        orientation[0] = orientation[0] - np.pi/2
        orientation[1] = orientation[1] - np.pi/2
        orientation[2] = orientation[2] - np.pi/2
        print(orientation)
        # Generate new pose 10cm away from pose in the opposite dir of orientation
        new_pose = deepcopy(curr_pose)
        new_pose.pose.position.x -= self.move_dist_from_pull_release * np.cos(orientation[1]) * np.sin(orientation[2])
        new_pose.pose.position.y -= self.move_dist_from_pull_release * np.cos(orientation[1]) * np.cos(orientation[2])
        new_pose.pose.position.z -= self.move_dist_from_pull_release * np.sin(orientation[1])
        print(new_pose)
        raw_input("Check")
        return new_pose


    def generate_trajectory_straight_back(self, curr_pose):
        end_pose = curr_pose
        end_pose.pose.position.x = self.arm_poses["start_pose"]["position"]["x"]
        fraction = 0
        while fraction < 0.99:
            trajectory, fraction = self.arm_group.compute_cartesian_path(
                waypoints=[end_pose.pose],
                eef_step=0.01,
                jump_threshold=2
            )
        return trajectory


    def run_arm_to_pose_straight_back(self, curr_pose=None):
        if curr_pose is None:
            curr_pose = self.arm_group.get_current_pose()
        traj = self.generate_trajectory_straight_back(curr_pose)
        trajectory_completed = self.arm_group.execute(traj, wait=True)
        return


    def run_arm_to_joint_orientation(self, joint_angles):
        """Run the arm to a specific joint orientation"""
        self.arm_group.set_joint_value_target(joint_angles)
        self.arm_group.plan()
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        self.current_joint_values = self.arm_group.get_current_joint_values()  # How to get current joint positions
        # print("Joint angles", self.current_joint_values)
        return


    def run_arm_to_start_pose(self):
        """Run the arm to the starting pose"""
        return self.run_arm_to_joint_orientation(self.start_pose)
        # self.target_pose = self.start_pose
        # return self.run_arm_to_target_pose()


    def run_arm_to_target_pose(self, target):
        """Run the end effector to a specific position and orientation"""
        self.arm_group.set_pose_target(target, end_effector_link="tool_frame")
        out = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        # print "Reached target:\n", self.target_pose
        return


    def run_arm_to_cartesian_pose(self, pose=None):
        if pose is None:
            grasp_pose = self.generate_pose(
                position=self.arm_poses["grasp_pose"]["position"],
                orientation=self.arm_poses["grasp_pose"]["orientation"]
            )
        else:
            grasp_pose = pose
        fraction = 0
        planning_start_time = time.time()
        while fraction <= 0.99:
            trajectory, fraction = self.arm_group.compute_cartesian_path(
                waypoints=[grasp_pose],
                eef_step=0.01,
                jump_threshold=2
            )
            if time.time() - planning_start_time > 60:
                return 1 # return error, move to next
        trajectory_completed = self.arm_group.execute(trajectory, wait=True)
        return 0
    
    def run_waypoints(self, pose_list):
        fraction = 0
        planning_start_time = time.time()
        while fraction <= 0.99:
            trajectory, fraction = self.arm_group.compute_cartesian_path(
                waypoints=pose_list,
                eef_step=0.01,
                jump_threshold=5.0
            )
            if time.time() - planning_start_time > 60:
                return 1 # return error, move to next
        trajectory_completed = self.arm_group.execute(trajectory, wait=True)
        return 0


    def run_once(self, pose=None):
        # Make sure gripper is open
        self.gripper_group.set_named_target("Open")
        self.gripper_group.go(wait=True)

        # Move to start position
        self.run_arm_to_start_pose()
        
        # Generate pose or use existing, move to pose
        if pose is None:
            pose = self.generate_pose(
                position=self.arm_poses["grasp_pose"]["position"],
                orientation=self.arm_poses["grasp_pose"]["orientation"]
            )

        # Add handle constraint
        self._add_constraints({"obj": self.door_env_constraints[self.grasp_obj_type]})

        # Run to grasping pose
        plan_fail = self.run_arm_to_cartesian_pose(pose)
        if plan_fail:
            print("Planning for grasp failed. Moving to next pose.")
            # Open gripper
            self.gripper_group.set_named_target("Open")
            self.gripper_group.go(wait=False)
            time.sleep(3)
            self.gripper_group.stop()
            return

        # Remove handle/knob from scene
        self.scene.remove_world_object(name=self.grasp_obj_type)

        # Close gripper
        self.gripper_group.set_named_target("Close")
        self.gripper_group.go(wait=False)
        time.sleep(3)
        self.gripper_group.stop()


        # Pull door back
        curr_pose = self.arm_group.get_current_pose()
        self.run_arm_to_pose_straight_back(curr_pose)

        # Open gripper
        self.gripper_group.set_named_target("Open")
        self.gripper_group.go(wait=False)
        time.sleep(3)
        self.gripper_group.stop()

        # Move arm out of grasp object space
        move_away_pose = self.generate_trajectory_away_from_grasp_obj()
        plan_fail = self.run_arm_to_cartesian_pose(pose=move_away_pose.pose)
        if plan_fail:
            print("Planning for grasp failed. Moving to next pose.")
            return
        return

    
    def run_pose_list(self):
        # poses = self.generate_poses_YTRANS(direction=-1)
        # poses = self.generate_poses_YROT()
        # poses = self.generate_poses_ZROT(direction=-1)
        #poses = self.generate_poses_YROT(direction=-1)
        # print(poses)

        for i, pose in enumerate(poses):
            print("Starting run number {}".format(i))
            print(pose)
            self.run_once(pose=pose)
            raw_input("Press enter for next run.")       
        return


    def open_close_gripper(self):
        while True:
            self.gripper_group.set_named_target("Open")
            self.gripper_group.go(wait=False)
            time.sleep(2)
            raw_input("Press enter to close gripper.")
            self.gripper_group.set_named_target("Close")
            self.gripper_group.go(wait=False)
            time.sleep(2)
            raw_input("Press enter to open gripper.")
        return


def main():
    rospy.init_node("door_arm_controller_gen_3", argv=sys.argv)
    
    dac = DoorArmController("handle")

    #dac.pose_list = list(dac.generate_poses_YROT(direction=1))
    # dac.pose_list = list(dac.generate_poses_ZROT(direction=-1))
    # dac.run_pose_list()
    # dac.run_once()
    rospy.spin()
    return

if __name__ == "__main__":
    main()