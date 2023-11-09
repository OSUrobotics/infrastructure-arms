#!/usr/bin/env python2
"""
drawer_trial_path_planner_kinova_jayco_2.py
Author: Luke Strohbehn
Email: strohbel@oregonstate.edu
Date: 08/20/2023
"""

import sys
import subprocess
import rospy
import actionlib
import time
from infrastructure_msgs.msg import StageAction, StageGoal, StageFeedback, StageResult, ArmTrialMetadata
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Pose
import std_msgs
import moveit_commander
import tf.transformations as tfs
import numpy as np
import json
import os
import pprint
from copy import deepcopy



class DrawerArmController:
    def __init__(self, grasp_obj_type):
        self.grasp_obj_type = grasp_obj_type.strip().lower()
        # Joint scene JSON info
        __here__ = os.path.dirname(__file__)
        # Base environment objects
        drawer_env_base_path = os.path.join(__here__, "drawer", "drawer_env_base.json")
        with open(drawer_env_base_path, "r") as jfile:
            self.drawer_env_constraints = json.load(jfile)
        # Load handle/knob type object
        grasp_obj_path = os.path.join(__here__, "drawer", self.grasp_obj_type+".json")
        with open(grasp_obj_path, "r") as jfile:
            grasp_obj = json.load(jfile)
            # Add handle/knob to constraints
            self.drawer_env_constraints[grasp_obj[self.grasp_obj_type]["name"]] = grasp_obj[self.grasp_obj_type]

        # Load preset poses
        arm_poses_path = os.path.join(__here__,"joint_angles", "drawer.json") 
        with open(arm_poses_path, "r") as jfile:
            self.arm_poses = json.load(jfile)

        # Initialize Metadata publisher, reset metadata subscriber
        self.metadata_pub = rospy.Publisher("/arm_trial_metadata", ArmTrialMetadata, queue_size=10)
        self.reset_metadata_sub = rospy.Subscriber('/reset_metadata', std_msgs.msg.Float32, self.force_callback)

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
        # self.move_group.set_num_planning_attempts(3)
        # self.move_group.set_timeout()

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

        # Start pose
        self.start_pose = list(map(
            lambda k: self.arm_poses["initial_joint_position"][k],
            sorted(self.arm_poses["initial_joint_position"].keys())
        ))
        # self.start_pose = self.generate_pose(
        #     position = self.arm_poses["start_pose"]["position"],
        #     orientation = self.arm_poses["start_pose"]["orientation"]
        # )
        self.target_pose = self.start_pose
        self.pose_list = []
        self.move_dist_from_pull_release = 0.05

        rospy.sleep(3)
        self.joint_angle_rounded = 2
        return      
   

    def start_arm_sequence_callback(self, goal):
        # Start arm publisher
        self.start_arm.publish_feedback(StageFeedback(status="EXAMPLE: GRABBING OBJECT"))
        
        rospy.sleep(1.0)
        try:
            # Pop new grasp pose from list
            run_pose = self.pose_list.pop(0)
            # run_pose = self.pose_list[0]
            # Publish metadata
            metadata_msg = ArmTrialMetadata()
            metadata_msg.arm = "Kinova Jaco2"
            metadata_msg.gripper = "Jaco2 Gripper"
            metadata_msg.grasp_pose = run_pose
            metadata_msg.pull_type = "linear_x"
            self.metadata_pub.publish(metadata_msg)
            
            self.run_once(run_pose)
            self.start_arm.set_succeeded(StageResult(result=0), text="SUCCESS")
        except IndexError:
            print("Trial sequence ended, no more poses to run.")
        return


    def force_callback(self, force):
        self.trial_force = force
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
    

    def generate_poses_XTRANS(self, step_size=0.01, num=10, direction=1):
        grasp_pose = self.generate_pose(
            position=self.arm_poses["grasp_pose"]["position"],
            orientation=self.arm_poses["grasp_pose"]["orientation"]
        )
        pose_list = np.empty(num, dtype=Pose)

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

        pose_list = np.empty(num, dtype=Pose)

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

        pose_list = np.empty(num, dtype=Pose)

        for i in range(num):
            modified_pose = deepcopy(grasp_pose)
            modified_pose.position.z += step_size * i * direction
            pose_list[i] = modified_pose
        return pose_list
    

    def generate_poses_XROT(self, step_size=0.05, num=10, direction=1):
        pose_list = np.empty(num, dtype=Pose)
        for i in range(num):
            modified_pose = deepcopy(self.arm_poses["grasp_pose"])
            modified_pose["orientation"]["x"] += np.degrees(step_size * i * direction)
            pose_list[i] = self.generate_pose(
                position=modified_pose["position"],
                orientation=modified_pose["orientation"]
            )
        return pose_list


    def generate_poses_YROT(self, step_size=0.05, num=10, direction=1):
        pose_list = np.empty(num, dtype=Pose)
        for i in range(num):
            modified_pose = deepcopy(self.arm_poses["grasp_pose"])
            modified_pose["orientation"]["y"] += np.degrees(step_size * i * direction)
            pose_list[i] = self.generate_pose(
                position=modified_pose["position"],
                orientation=modified_pose["orientation"]
            )
        return pose_list
    

    def generate_poses_ZROT(self, step_size=0.05, num=10, direction=1):
        pose_list = np.empty(num, dtype=Pose)
        for i in range(num):
            modified_pose = deepcopy(self.arm_poses["grasp_pose"])
            modified_pose["orientation"]["z"] += np.degrees(step_size * i * direction)
            pose_list[i] = self.generate_pose(
                position=modified_pose["position"],
                orientation=modified_pose["orientation"]
            )
        return pose_list


    def generate_trajectory_away_from_grasp_obj(self):
        curr_pose = self.move_group.get_current_pose()
        print(curr_pose)
        orientation = list(tfs.euler_from_quaternion([
            curr_pose.pose.orientation.x,
            curr_pose.pose.orientation.y,
            curr_pose.pose.orientation.z,
            curr_pose.pose.orientation.w
        ]))
        # Put y rotation into format I did my math in
        orientation[0] = orientation[0] #+ np.pi
        orientation[1] = orientation[1] - np.pi/2
        orientation[2] = orientation[2] #+ np.pi
        print(orientation)
        # Generate new pose 10cm away from pose in the opposite dir of orientation
        new_pose = deepcopy(curr_pose)
        # if orientation[2] > -np.pi:
        #     new_pose.pose.position.x += self.move_dist_from_pull_release * np.cos(orientation[1]) * np.sin(orientation[2])
        # else:
        new_pose.pose.position.x -= abs(self.move_dist_from_pull_release * np.cos(orientation[1]) * np.cos(orientation[2]))
        new_pose.pose.position.y += self.move_dist_from_pull_release * np.cos(orientation[1]) * np.sin(orientation[2])
        new_pose.pose.position.z += self.move_dist_from_pull_release * np.sin(orientation[1])
        print(new_pose)
        # raw_input("Check")
        return new_pose


    def generate_list_of_pose(self, pose, num=10):
        pose_list = np.empty(num, dtype=Pose)
        for i in range(num):
            pose_list[i] = pose
        return pose_list


    def generate_trajectory_straight_back(self, curr_pose):
        # end_pose = curr_pose
        # end_pose.pose.position.x = self.arm_poses["start_pose"]["position"]["x"]
        # fraction = 0
        # planning_start_time = time.time()
        # while fraction < 0.99: # TODO: just call run_arm_to_cartesian_pose
        #     trajectory, fraction = self.move_group.compute_cartesian_path(
        #         waypoints=[end_pose.pose],
        #         eef_step=0.01,
        #         jump_threshold=2
        #     )
        #     if time.time() - planning_start_time > 60:
        #         return 1 # return error, move to next

        end_pose = curr_pose
        # if curr_pose.pose.position.x - 0.1 > self.arm_poses["start_pose"]["position"]["x"]:
        #     end_pose.pose.position.x = curr_pose.pose.position.x - 0.1
        #     print("Went with modified pose x")
        # else:
        #     end_pose.pose.position.x = self.arm_poses["start_pose"]["position"]["x"]
        #     print("Went with start_pose x")
        end_pose.pose.position.x = curr_pose.pose.position.x - 0.12
        print(end_pose.pose.position.x)
        fraction = 0
        planning_start_time = time.time()
        while fraction < 0.99: # TODO: just call run_arm_to_cartesian_pose
            trajectory, fraction = self.move_group.compute_cartesian_path(
                waypoints=[end_pose.pose],
                eef_step=0.01,
                jump_threshold=2
            )
            if time.time() - planning_start_time > 60:
                return 1 # return error, move to next
        return trajectory


    def run_arm_to_pose_straight_back(self, curr_pose=None):
        if curr_pose is None:
            curr_pose = self.move_group.get_current_pose()
        traj = self.generate_trajectory_straight_back(curr_pose)
        trajectory_completed = self.move_group.execute(traj, wait=True)
        return


    def run_arm_to_joint_orientation(self, joint_angles):
        """Run the arm to a specific joint orientation"""
        self.move_group.set_joint_value_target(joint_angles)
        self.move_group.plan()
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        self.current_joint_values = self.move_group.get_current_joint_values()  # How to get current joint positions
        # print("Joint angles", self.current_joint_values)
        return


    def run_arm_to_start_pose(self):
        """Run the arm to the starting pose"""
        return self.run_arm_to_joint_orientation(self.start_pose)
        # self.target_pose = self.start_pose
        # return self.run_arm_to_target_pose()


    def run_arm_to_target_pose(self):
        """Run the end effector to a specific position and orientation"""
        self.move_group.set_pose_target(self.target_pose)
        out = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
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
            trajectory, fraction = self.move_group.compute_cartesian_path(
                waypoints=[grasp_pose],
                eef_step=0.01,
                jump_threshold=2
            )
            if time.time() - planning_start_time > 60:
                return 1 # return error, move to next
        trajectory_completed = self.move_group.execute(trajectory, wait=True)
        return 0


    def run_once(self, pose=None):
        # Add handle constraint
        self._add_constraints({"obj": self.drawer_env_constraints[self.grasp_obj_type]})

        # Make sure gripper is open
        self.move_gripper.set_named_target("Open")
        self.move_gripper.go(wait=True)

        # Move to start position
        self.run_arm_to_start_pose()
        
        # Generate pose or use existing, move to pose
        if pose is None:
            pose = self.generate_pose(
                position=self.arm_poses["grasp_pose"]["position"],
                orientation=self.arm_poses["grasp_pose"]["orientation"]
            )

        # Remove handle/knob from scene
        self.scene.remove_world_object(name=self.grasp_obj_type)

        # Run to grasping pose
        plan_fail = self.run_arm_to_cartesian_pose(pose)
        if plan_fail:
            print("Planning for grasp failed. Moving to next pose.")
            # Open gripper
            self.move_gripper.set_named_target("Open")
            self.move_gripper.go(wait=False)
            time.sleep(3)
            self.move_gripper.stop()

            # Add handle constraint
            self._add_constraints({"obj": self.drawer_env_constraints[self.grasp_obj_type]})
            return

        

        # Close gripper
        self.move_gripper.set_named_target("Close")
        self.move_gripper.go(wait=False)
        time.sleep(3)
        self.move_gripper.stop()
        current_gripper_pose = self.move_gripper.get_current_joint_values()
        print(current_gripper_pose)
        while not np.all(np.array(current_gripper_pose) > 1.1):
            current_gripper_pose = self.move_gripper.get_current_joint_values()
            print(current_gripper_pose)
            self.move_gripper.set_named_target("Close")
            self.move_gripper.go(wait=False)
            time.sleep(1)


        # Pull drawer back
        curr_pose = self.move_group.get_current_pose()
        self.run_arm_to_pose_straight_back(curr_pose)

        # Open gripper
      
        self.move_gripper.set_named_target("Open")
        self.move_gripper.go(wait=False)
        time.sleep(3)
        self.move_gripper.stop()
        current_gripper_pose = self.move_gripper.get_current_joint_values()
        print("Current gripper: ", current_gripper_pose)
        while not np.all(np.array(current_gripper_pose) < .3):
            current_gripper_pose = self.move_gripper.get_current_joint_values()
            print(current_gripper_pose)
            self.move_gripper.set_named_target("Open")
            self.move_gripper.go(wait=False)
            time.sleep(1)

        # # Move arm out of grasp object space
        # move_away_pose = self.generate_trajectory_away_from_grasp_obj()
        # plan_fail = self.run_arm_to_cartesian_pose(pose=move_away_pose.pose)
        # if plan_fail:
        #     print("Planning for grasp failed. Moving to next pose.")
        #     return
        return

    
    def run_pose_list(self):
        # poses = self.generate_poses_YTRANS(direction=-1)
        # poses = self.generate_poses_YROT()
        # poses = self.generate_poses_ZROT(direction=-1)
        poses = self.generate_poses_YROT(direction=-1)
        # print(poses)

        for i, pose in enumerate(poses):
            print("Starting run number {}".format(i))
            print(pose)
            self.run_once(pose=pose)
            raw_input("Press enter for next run.")       
        return


    def open_close_gripper(self):
        while True:
            self.move_gripper.set_named_target("Open")
            self.move_gripper.go(wait=False)
            time.sleep(2)
            raw_input("Press enter to close gripper.")
            self.move_gripper.set_named_target("Close")
            self.move_gripper.go(wait=False)
            time.sleep(2)
            raw_input("Press enter to open gripper.")
        return


def main():
    rospy.init_node("drawer_arm_controller", argv=sys.argv)
    # dac = DrawerArmController("knob")
    dac = DrawerArmController("handle")

    # dac.pose_list = list(dac.generate_poses_YROT(direction=1))
    # dac.pose_list = list(dac.generate_poses_YROT(step_size=0.07853981633, direction=-1))
    # dac.pose_list = list(dac.generate_poses_ZROT(direction=-1))
    # dac.pose_list = list(dac.generate_poses_XTRANS(step_size=0.01, direction=-1))

    grasp_type = "palm_grasp_pose_knob"
    grasp_pose = dac.generate_pose(
                position=dac.arm_poses[grasp_type]["position"],
                orientation=dac.arm_poses[grasp_type]["orientation"]
            )
    dac.pose_list = list(dac.generate_list_of_pose(pose = grasp_pose))
    # dac.run_pose_list()
    # dac.run_once(pose=grasp_pose)
    # dac.run_arm_to_start_pose()
    # dac.target_pose = grasp_pose
    # dac.scene.remove_world_object(name=dac.grasp_obj_type)
    # dac.run_arm_to_target_pose()

    # rospy.signal_shutdown("Done.")
    # raw_input("Done.")
    rospy.spin()
    return

if __name__ == "__main__":
    main()