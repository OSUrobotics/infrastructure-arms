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
from trajectory_msgs.msg import JointTrajectory
import moveit_commander
from time import sleep
from tf.transformations import *
import numpy as np
import math
import kinova_msgs.msg
from copy import deepcopy
from sensor_msgs.msg import JointState
from kinova_msgs.srv import *
import kinova_msgs.msg

class DoorArmController():
    
    def __init__(self):   

        self.currentJointCommand=[0,0,0,0,0,0,0]
        #initializing actionservers
        self.start_arm = actionlib.SimpleActionServer("start_arm_sequence", StageAction, self.start_arm_sequence_callback, False) 
        self.start_arm.start()
        
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

        topic_address = '/' + "j2s7s300_" + 'driver/out/joint_command'
        rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, self.setCurrentJointCommand)
        rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)
        # Set up action server to directly send joint angles to the arm 
        action_address = '/' + "j2s7s300_" + 'driver/joints_action/joint_angles'
        self.client = actionlib.SimpleActionClient(action_address,
                                            kinova_msgs.msg.ArmJointAnglesAction)
        self.client.wait_for_server()
        rospy.loginfo("Server 1 up")
        
        # Set up action server to directly send joint angles to the arm 
        action_address = "/" + "j2s7s300" + "_driver/trajectory_controller"
        self.client_trajectory = actionlib.SimpleActionClient(action_address,
                                            kinova_msgs.msg.SetJointTrajectoryAction)
        #self.client_trajectory.wait_for_server()
        

        
        
        
        rospy.loginfo("Kinova Service is up!")
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
        # To see the trajectory
        self.disp = DisplayTrajectory()
        self.disp.trajectory_start = self.robot.get_current_state()
        self.rate = rospy.Rate(10)
        self.move_group.allow_replanning(1)

        # Add constraint areas 
        self.add_constraint_box(pos=[.63,0,1], dim=[.1,2,2], name="door")
        self.add_constraint_box(pos=[0,0,-.05], dim=[2,2,.1], name="table")
        self.add_constraint_box(pos=[-.53,.762,.5], dim=[.1,.25,1], orient=[0.0,0.0,1.0,.5], name="rear_left_camera")
        self.add_constraint_box(pos=[-.53,-.762,.5], dim=[.1,.25,1], orient=[0.0,0.0,1.0,-.5], name="rear_right_camera")
        self.add_constraint_box(pos=[.475,0,1.1], dim=[.2,2,.1], orient=[0.0,1.0,0.0,-.1], name="overhead_light")
        self.add_constraint_box(pos=[.46,-0.09,.28], dim=[.07,.07,.25], name="handle")
        rospy.loginfo("Added scene constraints.")


        """

        topic_name = '/' + "j2s7s300_" + 'driver/out/joint_state'
        sub = rospy.Subscriber(topic_name, JointState, self.getFeedbackCallback)








        max_error = [0,0,0,0,0,0,0]
        counter = [0]

        topic_name = '/' + "j2s7s300_" + 'driver/in/joint_velocity'
        pub = rospy.Publisher(topic_name, kinova_msgs.msg.JointVelocity, queue_size=1)
        jointCmd = kinova_msgs.msg.JointVelocity()
        speed = .1
        jointCmd.joint1 = speed
        jointCmd.joint2 = 0
        jointCmd.joint3 = 0
        jointCmd.joint4 = 0
        jointCmd.joint5 = 0
        jointCmd.joint6 = 0
        jointCmd.joint7 = 0
        count = 0	
        pub.publish(jointCmd)
        	
        rate = rospy.Rate(100)
        while (count < 500):
            count = count + 1
            #rospy.loginfo("I will publish to the topic %d", count)
            pub.publish(jointCmd)
            rate.sleep()
            print("here")

        sub.unregister()
        """
        

        

        # Wait for user before moving to the starting position
        raw_input("Press enter to go to starting position.")
        # Open gripper and send to home position
        self.move_gripper.set_named_target("Open")
        self.move_gripper.go(wait=True)
        self.send_joint_angle_target([-0.780338353837345967,2.540778717977955,0.4260529439545537,5.5358591620936926,5.08,4.417817531159352,0.057345469084458735])        
        


        user = raw_input("done, press enter to continue")
      
        #
        self.remove_box("handle")

        rospy.loginfo("Added scene constraints.")

        rospy.sleep(3)
        #how much to round joint angles
        counter = 0
        while True:
            # Wait till we get a plan with 100%
            print("Counter: ", counter)
            counter += 1
            
            pose_list = [self.generate_pose([.46,-.1,.32], [90, 0, 50])]
            
            plan, fraction = self.move_group.compute_cartesian_path(pose_list, 0.005,5)
            print("Fraction", fraction)
            if fraction > .99:
                break
        self.current_joint_values = self.move_group.get_current_joint_values() # How to get current joint positions
        #print("Moveit: ", self.current_joint_values)
        #print("Kinova: ", self.currentJointCommand)
        #print("Plan: ",plan.joint_trajectory.points[0].positions[0])

        #print("Current pose: ", self.move_group.get_current_pose())

        for i in range(len(plan.joint_trajectory.points)):
            
            self.moveit_to_kinova(self.validate_move(plan.joint_trajectory.points[i].positions))
            print("a goal completed")

        self.current_joint_values = self.move_group.get_current_joint_values()
        print("Current pose: ", self.move_group.get_current_pose())

        print(self.move_gripper.get_current_joint_values())
        how = raw_input("Moving to move it start: ")
        self.joint_angle_rounded = 2 

        self.current_joint_values = self.move_gripper.get_current_joint_values()
        self.move_gripper.set_named_target("Close")
        self.move_gripper.go(wait=True)

        print("Moveit: ", self.current_joint_values)
        print("Kinova: ", np.radians(self.currentJointCommand))
        #pose_list = [self.generate_pose([.4,-.05,.32], [90, 0, 50]), self.generate_pose([.45,-.1,.32], [90, 0, 50])]
        counter = 0
        while True:
            # Wait till we get a plan with 100%
            print("Counter: ", counter)
            counter += 1
            
            pose_list = [self.generate_pose([.42,-.07,.32], [90, 0, 50]), self.generate_pose([.37,-.03,.32], [90, 0, 47]), self.generate_pose([.35, 0,.32], [90, 0, 43]), self.generate_pose([.33, .03,.32], [90, 0, 40])]#, self.generate_pose([.37,.1,.32], [90, 0, 50])]
            
            plan, fraction = self.move_group.compute_cartesian_path(pose_list, 0.005,5)
            print("Fraction", fraction)
            if fraction > .99:
                break
        self.current_joint_values = self.move_group.get_current_joint_values() # How to get current joint positions
        #print("Moveit: ", self.current_joint_values)
        #print("Kinova: ", self.currentJointCommand)
        #print("Plan: ",plan.joint_trajectory.points[0].positions[0])

        #print("Current pose: ", self.move_group.get_current_pose())

        for i in range(len(plan.joint_trajectory.points)):
            
            self.moveit_to_kinova(self.validate_move(plan.joint_trajectory.points[i].positions))
            print("a goal completed")

        self.current_joint_values = self.move_group.get_current_joint_values()
        print("Current pose: ", self.move_group.get_current_pose())
        """
        pose_list = [self.generate_pose([.35,.3,.32], [90, 0, 50])]#, self.generate_pose([.37,.1,.32], [90, 0, 50])]

        plan, fraction = self.move_group.compute_cartesian_path(pose_list, 0.005,4)
        self.current_joint_values = self.move_group.get_current_joint_values() # How to get current joint positions
        #print("Moveit: ", self.current_joint_values)
        #print("Kinova: ", self.currentJointCommand)
        #print("Plan: ",plan.joint_trajectory.points[0].positions[0])

        #print("Current pose: ", self.move_group.get_current_pose())

        for i in range(len(plan.joint_trajectory.points)):
            
            self.moveit_to_kinova(self.validate_move(plan.joint_trajectory.points[i].positions))
            print("a goal completed")

        self.current_joint_values = self.move_group.get_current_joint_values()
        print("Current pose: ", self.move_group.get_current_pose())
        """

        self.move_gripper.set_named_target("Open")
        self.move_gripper.go(wait=True)

        uh = raw_input("next")
        #print("Current pose: ", self.move_group.get_current_pose())
        pose_list = [self.generate_pose([.35,.1,.32], [90, 0, 50])]#, self.generate_pose([.37,.1,.32], [90, 0, 50])]
        plan, fraction = self.move_group.compute_cartesian_path(pose_list, 0.005,1)
        self.current_joint_values = self.move_group.get_current_joint_values() # How to get current joint positions
        print("Moveit: ", self.current_joint_values)
        print("Kinova: ", np.radians(self.currentJointCommand))
        #print("Plan: ",plan.joint_trajectory.points[0].positions[0])

       

        for i in range(len(plan.joint_trajectory.points)):
            
            self.moveit_to_kinova(self.validate_move(plan.joint_trajectory.points[i].positions))
            print("a goal completed")
        print("Current pose: ", self.move_group.get_current_pose())

        rospy.signal_shutdown("ah")
        ug = input("waiting")
 


        self.current_pose = self.move_group.get_current_pose() # How to get current pose


        print("Updated euler pose", euler_from_quaternion([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y,self.current_pose.pose.orientation.z,self.current_pose.pose.orientation.w]))
        #print(Pose())

    def send_joint_angle_target(self, joint_target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_allowed = 5.0):
        """ Passes a joint angle target to MoveIT! to generate a safe JointTrajectory. Validates path with validate_move(), then sends to the robot via the Kinova action server.

        Args:
            joint_target (1x7 list): Target joint positions for all 7 joints.
                (default is [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            time_allowed (foat): Amount of time allowed in seconds per individual move before timeout.
                (default is 5.0 seconds) 
        Returns:
            none
        """
                
        self.current_joint_values = self.move_group.get_current_joint_values()
        # Set the target and plan 
        self.move_group.set_joint_value_target(joint_target)
        plan = self.move_group.plan()
        plan.joint_trajectory
        self.client_trajectory.send_goal(plan.joint_trajectory)

        if self.client_trajectory.wait_for_result(rospy.Duration(200.0)):
            rospy.loginfo("THIS WORKEEEDDDD")
            
        else:
            self.client_trajectory.cancel_all_goals()
            rospy.logerr("AHHH NOOOO")
        
        """

        # Loop through the joint trajectory and send to the Kinova
        for i in range(len(plan.joint_trajectory.points)):
            self.moveit_to_kinova(self.validate_move(plan.joint_trajectory.points[i].positions), duration = time_allowed)
        
        rospy.loginfo("Complete joint angle move.")
        """

    
    def getFeedbackCallback(self, in_msg):
        print("Got data")
        print(in_msg)


    def validate_move(self, positions_tuple):
        joint_threshold = .4

        current_joint_values = deepcopy([math.radians(self.currentJointCommand[0]),math.radians(self.currentJointCommand[1]),math.radians(self.currentJointCommand[2]),math.radians(self.currentJointCommand[3]),math.radians(self.currentJointCommand[4]),math.radians(self.currentJointCommand[5]),math.radians(self.currentJointCommand[6])])

        positions = np.array(positions_tuple)
        #current_joint_values = self.move_group.get_current_joint_values() 
        #print("Target positions: ", positions)
        #print("Current angles: ", current_joint_values)
        # Check that target joint positions are close to current
        for i, pos in enumerate(positions):
            diff = pos-current_joint_values[i]
            if np.abs(pos-current_joint_values[i]) < joint_threshold:
                continue
            else:
                # Too large! Check if a multiple of 2*pi
                if (diff - np.pi*2) < joint_threshold and (diff - np.pi*2) > -joint_threshold:
                    # If a multiple of 2pi, subtract 2pi
                    positions[i] -= np.pi*2
                elif (diff + np.pi*2) < joint_threshold and (diff + np.pi*2) > -joint_threshold:
                    # If a multiple of 2pi, add 2pi
                    positions[i] += np.pi*2
                elif (diff - np.pi*4) < joint_threshold and (diff - np.pi*4) > -joint_threshold:
                    # If a multiple of 4pi, subtract 4pi
                    positions[i] -= np.pi*4
                elif (diff + np.pi*4) < joint_threshold and (diff + np.pi*4) > -joint_threshold:
                    # If a multiple of 4pi, add 4pi
                    positions[i] += np.pi*4
                else: 
                    rospy.logerr("Target joint angle outside acceptable range")
                    return current_joint_values
        # After all joint angles are validated, return the target angles
        # print("Fixed positions: ", positions)
        return positions



    def moveit_to_kinova(self, joint_angles, duration = 5.0):
        #print("Positions: ", joint_angles)
        goal = kinova_msgs.msg.ArmJointAnglesGoal()
        angle_set = [math.degrees(joint_angles[0]), math.degrees(joint_angles[1]), math.degrees(joint_angles[2]), math.degrees(joint_angles[3]), math.degrees(joint_angles[4]), math.degrees(joint_angles[5]), math.degrees(joint_angles[6])]
      
        goal.angles.joint1 = angle_set[0]
        goal.angles.joint2 = angle_set[1]
        goal.angles.joint3 = angle_set[2]
        goal.angles.joint4 = angle_set[3]
        goal.angles.joint5 = angle_set[4]
        goal.angles.joint6 = angle_set[5]
        goal.angles.joint7 = angle_set[6]

        self.client.send_goal(goal)
        
        if self.client.wait_for_result(rospy.Duration(duration)):
            print(self.client.get_result())
        else:
            print('The joint angle action timed-out :(')
            self.client.cancel_all_goals()

    def setCurrentJointCommand(self, feedback):
        currentJointCommand_str_list = str(feedback).split("\n")
        for index in range(0,len(currentJointCommand_str_list)):
            temp_str=currentJointCommand_str_list[index].split(": ")
            self.currentJointCommand[index] = float(temp_str[1])

 
    def start_arm_sequence_callback(self, goal):
    
        self.start_arm.publish_feedback(StageFeedback(status="EXAMPLE: GRABBING OBJECT"))
        #user_n = raw_input("bro")
        # Do arm call here
        rospy.sleep(1.0)
        self.start_arm.set_succeeded(StageResult(result = 0), text="SUCCESS")

    def generate_pose(self, position, orientation):
        quat = quaternion_from_euler(math.radians(orientation[0]),math.radians(orientation[1]),math.radians(orientation[2]))
        temp_pose = Pose()
        temp_pose.position.x = position[0]
        temp_pose.position.y = position[1]
        temp_pose.position.z = position[2]
        temp_pose.orientation.x = quat[0]
        temp_pose.orientation.y = quat[1]
        temp_pose.orientation.z = quat[2]
        temp_pose.orientation.w = quat[3]

        return temp_pose

    def modify_arm_pose_cartesian(self, position, orientation):
        # Takes in a position (m) and orientation (euler, degrees) and sends it the arm
        # first orientation, around z, second around y, third around x
        quat = quaternion_from_euler(math.radians(orientation[0]),math.radians(orientation[1]),math.radians(orientation[2]))
        goal_pose = Pose()
        goal_pose.position.x = position[0]
        goal_pose.position.y = position[1]
        goal_pose.position.z = position[2]
        goal_pose.orientation.x = quat[0]
        goal_pose.orientation.y = quat[1]
        goal_pose.orientation.z = quat[2]
        goal_pose.orientation.w = quat[3]
        self.move_group.set_pose_target(goal_pose)
        out = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()



    def add_constraint_box(self, name, dim, pos, orient=[0.0,0.0,1.0,0.0]):
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

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < 5) and not rospy.is_shutdown():
            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self.scene.get_known_object_names()
            # Test if we are in the expected state
            if is_known == True:
                return True
                print("added box")
            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()
            # If we exited the while loop without returning then we timed out
        print("failed adding box")
        return False
    
    def remove_box(self, name):
        self.scene.remove_world_object(name)
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < 5) and not rospy.is_shutdown():
            # Test if the box is in the scene.
            is_known = name in self.scene.get_known_object_names()
            if is_known == False:
                return True
            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False



if __name__ == '__main__':
    rospy.init_node("door_arm_controller_what", argv=sys.argv)
    begin = DoorArmController()
    rospy.spin()
