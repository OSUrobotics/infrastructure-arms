#!/usr/bin/env python2

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
import pprint


class DrawerArmController:
    def __init__(self, grasp_obj_type):
        # Joint scene JSON info
        __here__ = os.path.dirname(__file__)
        # Base environment objects
        drawer_env_base_path = os.path.join(__here__, "drawer", "drawer_env_base.json")
        with open(drawer_env_base_path, "r") as jfile:
            self.drawer_env_constraints = json.load(jfile)
        # Load handle/knob type object
        grasp_obj_path = os.path.join(__here__, "drawer", grasp_obj_type.strip().lower()+".json")
        with open(grasp_obj_path, "r") as jfile:
            grasp_obj = json.load(jfile)
            self.drawer_env_constraints[grasp_obj[grasp_obj_type]["name"]] = grasp_obj[grasp_obj_type]

        # Load preset poses
        arm_poses_path = os.path.join(__here__,"joint_angles", "drawer.json") 
        with open(arm_poses_path, "r") as jfile:
            self.arm_poses = json.load(jfile)
        

        # initializing actionservers
        self.start_arm = actionlib.SimpleActionServer(
            "start_arm_sequence", StageAction, self.start_arm_sequence_callback, False
        )
        self.start_arm.start()
        # print(real_robot)
        real_robot = True
        # To read from redirected ROS Topic (Gazebo launch use)
        if real_robot:
            joint_state_topic = ["joint_states:=/j2s7s300_driver/out/joint_state"]
            moveit_commander.roscpp_initialize(joint_state_topic)
            # rospy.init_node('move_kinova', anonymous=False)
            moveit_commander.roscpp_initialize(sys.argv)
        else:
            joint_state_topic = ["joint_states:=/j2s7s300/joint_states"]
            moveit_commander.roscpp_initialize(joint_state_topic)
            # rospy.init_node('move_kinova', anonymous=False)
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

        self.apply_scene = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)
        self.get_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
        # rospy.sleep(2)

        # To see the trajectory
        self.disp = DisplayTrajectory()

        self.disp.trajectory_start = self.robot.get_current_state()

        self.rate = rospy.Rate(10)

        self.move_group.allow_replanning(1)

        # Add constraint areas
        self._add_constraints(self.drawer_env_constraints)
        rospy.loginfo("Added scene constraints.")

        # # Start pose
        # self.start_pose = list(map(
        #     lambda k: self.arm_poses["initial_joint_position"][k],
        #     sorted(self.arm_poses["initial_joint_position"].keys())
        # ))
        self.start_pose = self.generate_pose(
            position = self.arm_poses["start_pose"]["position"],
            orientation = self.arm_poses["start_pose"]["orientation"]
        )

        rospy.sleep(3)
        self.joint_angle_rounded = 2

        
    def run_arm_to_joint_orientation(self, joint_angles):
        """Run the arm to a specific joint orientation"""
        self.move_group.set_joint_value_target(joint_angles)
        self.move_group.plan()
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        self.current_joint_values = self.move_group.get_current_joint_values()  # How to get current joint positions
        print("Joint angles", self.current_joint_values)
        return


    def run_arm_to_start_pose(self):
        """Run the arm to the starting pose"""
        # return self.run_arm_to_joint_orientation(self.start_pose)
        self.target_pose = self.start_pose
        return self.run_arm_to_target_pose()


    def run_arm_to_target_pose(self):
        """Run the end effector to a specific position and orientation"""
        self.move_group.set_pose_target(self.target_pose)
        out = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        print "Reached target:\n", self.target_pose
        return


    def start_arm_sequence_callback(self, goal):

        self.start_arm.publish_feedback(StageFeedback(status="EXAMPLE: GRABBING OBJECT"))
        # user_n = raw_input("bro")
        # Do arm call here
        rospy.sleep(1.0)
        self.start_arm.set_succeeded(StageResult(result=0), text="SUCCESS")
        return


    def modify_arm_pose_cartesian(self):
        #
        print("bro")
        return


    def generate_pose(self, position, orientation):
        """Generate a pose from a position and orientation
        @param position: position of the end effector in meters
        @param orientation: orientation of the end effector in degrees
        """
        temp_pose = Pose()
        try:
            temp_pose.position.x = position[0]
            temp_pose.position.y = position[1]
            temp_pose.position.z = position[2]
            quat = tfs.quaternion_from_euler(
                np.radians(orientation[0]),
                np.radians(orientation[1]),
                np.radians(orientation[2])
            )
        except KeyError:
            temp_pose.position.x = position["x"]
            temp_pose.position.y = position["y"]
            temp_pose.position.z = position["z"]
            quat = tfs.quaternion_from_euler(
                np.radians(orientation["x"]),
                np.radians(orientation["y"]),
                np.radians(orientation["z"])
            )
        temp_pose.orientation.x = quat[0]
        temp_pose.orientation.y = quat[1]
        temp_pose.orientation.z = quat[2]
        temp_pose.orientation.w = quat[3]

        return temp_pose
    

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
        box_pose.header.frame_id = "world"
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
        
        self.scene.add_box(box_name, box_pose, size=box_size)

        if self._check_object_presence(box_name):
            return True
        else:
            return False


    def _add_constraint_cylinder(self, cyl):
        """Add a constraint cylinder to the scene"""
        cyl_pose = PoseStamped()
        cyl_pose.header.frame_id = "world"
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

        self.scene.add_cylinder(cyl_name, cyl_pose, cyl_height, cyl_radius)

        if self._check_object_presence(cyl_name):
            return True
        else:
            return False


    def _check_object_presence(self, obj_name):
        """Check to see if the constraint object was successfully added"""
        start_time = time_now = rospy.get_time()
        while (time_now - start_time < 5) and not rospy.is_shutdown():
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


    def print_pose(self, pose):
        pose = {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w
            }
        }
        pprint.pprint(pose)
        return 


    def run(self):

        # Move to start position
        self.current_pose = self.move_group.get_current_pose()
        self.print_pose(self.current_pose.pose)
        self.run_arm_to_start_pose()
        curr_joint_vals = self.move_group.get_current_joint_values()
        print(curr_joint_vals)


        # # Go to standard pull position
        # self.current_pose = self.move_group.get_current_pose()
        # self.target_pose = self.generate_pose([0.37, 0.3, 0.31], [90, 0, 50])
        # self.run_arm_to_target_pose()

        rospy.signal_shutdown("done")
        hey = raw_input("arm at target pose")
        return


if __name__ == "__main__":
    rospy.init_node("drawer_arm_controller_what", argv=sys.argv)
    dac = DrawerArmController("knob")
    dac.run()
    rospy.spin()
