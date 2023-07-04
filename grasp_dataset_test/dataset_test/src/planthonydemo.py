#!/usr/bin/env python
#modified from example code in kortex_examples
import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import *
import math
import shape_msgs.msg
from copy import deepcopy
from dataset_test.srv import PlantLocation, PlantLocationResponse

class GraspDatasetTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(GraspDatasetTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')
    self.pose_list = []

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
      #self.remove_box("base")
      #self.remove_box("cube")
      #self.add_box(pos=[.5,0,.05], dim=[.04,.04,.1], name="cube")
      #self.remove_box("cam_stand")
      self.add_box(pos=[.5,0,.9], dim=[.1, .1, .5], name="cam_stand")
      #self.remove_box("backdrop")
      self.add_box(pos=[1,0,.5], dim=[.1, 2, 1], name="backdrop")
      #self.remove_box("top")
      self.add_box(pos=[0,0,1.1], dim=[1, 1, .1], name="top")
      self.add_box(pos=[.75,0,-.05], dim=[2,2,.1], name="base")

      self.planthony_srv = rospy.Service("planthony", PlantLocation, self.planthony_callback)


    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

    rospy.spin()



  def planthony_callback(self, request):
    actual_pose = self.get_cartesian_pose()
    print(request)
    actual_pose.position.x = request.loc.x
    actual_pose.position.y = request.loc.y
    actual_pose.position.z = request.loc.z
    self.reach_cartesian_pose(actual_pose, tolerance=.01, constraints=None)
    return PlantLocationResponse(1)


  
  def execute_poses(self):
    success = self.is_init_success
    self.pose_list = []
    self.generate_poses_top_XROT()
    self.generate_poses_top_ZROT()
    self.generate_poses_top_YROT()
    self.remove_box("cube")
    rospy.loginfo("Opening the gripper...")
    success &= self.reach_gripper_position(0)
    self.arm_group.set_end_effector_link("tool_frame")
 
    if success:
      for i in self.pose_list:
        self.add_box(pos=[.55,0,.05], dim=[.04,.04,.1], name="cube")
        if success:
          rospy.loginfo("Reaching Named Target Home...")
          success = self.reach_named_position("home")
        if success:
          rospy.loginfo("Moving to Pose: %s", str(i))
          success = self.reach_cartesian_pose(pose=i, tolerance=0.001, constraints=None)
          #success = self.plan_cartestian(pose=i)
        if success:
          self.remove_box("cube")
          rospy.sleep(1)
          rospy.loginfo("Closing the gripper...")
          success = self.reach_gripper_position(1)
        if success:
          success = self.set_cube_in_goal()
          print(success)
        if success:
          rospy.loginfo("Opening the gripper...")
          success = self.reach_gripper_position(0) 
      if success:
        rospy.loginfo("Reaching Named Target Home...")
        success = self.reach_named_position("home")

  def plan_cartestian(self, pose):
    waypoints = []
    waypoints.append(deepcopy(pose))
    plan, fraction = self.arm_group.compute_cartesian_path(waypoints, 0.01,0.0)    
    success = self.arm_group.execute(plan)
    return success
 

  def set_cube_in_goal(self):
    actual_pose = self.get_cartesian_pose()
    actual_pose.position.x = .58
    actual_pose.position.y = .25
    actual_pose.position.z = .1
    # constraints = moveit_msgs.msg.Constraints()
    # orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    # orientation_constraint.header.stamp = rospy.Time(0)
    # orientation_constraint.header.frame_id = "world"
    # orientation_constraint.link_name = "tool_frame"
    # orientation_constraint.absolute_x_axis_tolerance = .1
    # orientation_constraint.absolute_y_axis_tolerance = .1
    # orientation_constraint.absolute_z_axis_tolerance = .1
    # orientation_constraint.weight = 1
    # orientation_constraint.orientation = actual_pose.orientation
    # #constraint
    # position_constraint = moveit_msgs.msg.PositionConstraint()
    # position_constraint.header.stamp = rospy.Time(0)
    # position_constraint.header.frame_id = "world"
    # position_constraint.link_name = "tool_frame"
    # position_constraint.target_point_offset.x = 0
    # position_constraint.target_point_offset.y = 0
    # position_constraint.target_point_offset.z = 0
    # #box shape
    # box_ = shape_msgs.msg.SolidPrimitive()
    # box_.type = 1
    # box_.dimensions = [1,2,.65]
    # shape_list = [box_]
    # #box location
    # box_pose = geometry_msgs.msg.Pose()
    # box_pose.position.x = .5
    # box_pose.position.y = 0
    # box_pose.position.z = .325
    # shape_pos = [box_pose]
    # position_constraint.constraint_region.primitives = shape_list
    # position_constraint.constraint_region.primitive_poses = shape_pos
    # position_constraint.weight = 2
    # constraints.orientation_constraints.append(orientation_constraint)
    # constraints.position_constraints.append(position_constraint)
    waypoints = []
    waypoints.append(deepcopy(actual_pose))
    plan, fraction = self.arm_group.compute_cartesian_path(waypoints, 0.01,0.0)    
    success = self.arm_group.execute(plan)
    
    #success = self.reach_cartesian_pose(pose=actual_pose, tolerance=0.02, constraints=None)
    #self.arm_group.clear_path_constraints()
    return success

  def generate_poses_top_YROT(self, lower=0, upper=90, start_point=[.55,0,.09], num=5):
    step_size = (abs(upper) + abs(lower)) / num
    for i in range(num+1):
      actual_pose = deepcopy(self.get_cartesian_pose())
      actual_pose.position.x = start_point[0]
      actual_pose.position.y = start_point[1]
      actual_pose.position.z = start_point[2]
      #rotate end effector facing table
      q_down = quaternion_from_euler(0,math.radians(lower + (step_size * i)),0)
      current_q = [actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w]
      q_new = quaternion_multiply(q_down, current_q)
      actual_pose.orientation.x = q_new[0]
      actual_pose.orientation.y = q_new[1]
      actual_pose.orientation.z = q_new[2]
      actual_pose.orientation.w = q_new[3]
      self.pose_list.append(actual_pose)


  def generate_poses_top_ZROT(self, lower=-20, upper=20, start_point=[.55,0,.1], num=5):
    step_size = (abs(upper) + abs(lower)) / num
    for i in range(num+1):
      actual_pose = deepcopy(self.get_cartesian_pose())
      actual_pose.position.x = start_point[0]
      actual_pose.position.y = start_point[1]
      actual_pose.position.z = start_point[2]
      #rotate end effector facing table
      q_down= quaternion_from_euler(0, 1.5708, 0) 
      current_q = [actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w]
      q_new = quaternion_multiply(q_down, current_q)
      q_x = quaternion_from_euler(0,0,math.radians(lower + (step_size * i)))
      prev_q = [q_new[0], q_new[1], q_new[2], q_new[3]]
      q_new = quaternion_multiply(q_x, prev_q)
      actual_pose.orientation.x = q_new[0]
      actual_pose.orientation.y = q_new[1]
      actual_pose.orientation.z = q_new[2]
      actual_pose.orientation.w = q_new[3]
      self.pose_list.append(actual_pose)

  def generate_poses_top_XROT(self, lower=-20, upper=20, start_point=[.55,0,.09], num=5):
    #Get up and down case
    actual_pose = deepcopy(self.get_cartesian_pose())
    actual_pose.position.x = start_point[0]
    actual_pose.position.y = start_point[1]
    actual_pose.position.z = start_point[2]
    #rotate end effector facing table
    q_down= quaternion_from_euler(0, 1.5708, 0) 
    current_q = [actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w]
    q_new = quaternion_multiply(q_down, current_q)
    actual_pose.orientation.x = q_new[0]
    actual_pose.orientation.y = q_new[1]
    actual_pose.orientation.z = q_new[2]
    actual_pose.orientation.w = q_new[3]
    self.pose_list.append(actual_pose)
 
    step_size = (abs(upper) + abs(lower)) / num
    for i in range(num+1):
      actual_pose = deepcopy(self.get_cartesian_pose())
      actual_pose.position.x = start_point[0]
      actual_pose.position.y = start_point[1]
      actual_pose.position.z = start_point[2]
      #rotate end effector facing table
      q_down= quaternion_from_euler(0, 1.5708, 0) 
      current_q = [actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w]
      q_new = quaternion_multiply(q_down, current_q)
      q_x = quaternion_from_euler(math.radians(lower + (step_size * i)), 0, 0)
      prev_q = [q_new[0], q_new[1], q_new[2], q_new[3]]
      q_new = quaternion_multiply(q_x, prev_q)
      actual_pose.orientation.x = q_new[0]
      actual_pose.orientation.y = q_new[1]
      actual_pose.orientation.z = q_new[2]
      actual_pose.orientation.w = q_new[3]
      self.pose_list.append(actual_pose)
     


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

  def add_box(self, dim, pos, name):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = pos[0] # slightly above the end effector
    box_pose.pose.position.y = pos[1] # slightly above the end effector    
    box_pose.pose.position.z = pos[2] # slightly above the end effector
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
      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    # If we exited the while loop without returning then we timed out
    return False

  def reach_named_position(self, target):
    arm_group = self.arm_group
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    planned_path1 = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(planned_path1, wait=True)

  def reach_joint_angles(self, tolerance, joint_angles):  
    arm_group = self.arm_group
    success = True
    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)
    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)
    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = joint_angles[0]
      joint_positions[1] = joint_angles[1]
      joint_positions[2] = joint_angles[2]
      joint_positions[3] = joint_angles[3]
      joint_positions[4] = joint_angles[4]
      joint_positions[5] = joint_angles[5]
      joint_positions[6] = joint_angles[6]
    arm_group.set_joint_value_target(joint_positions)
    # Plan and execute in one command
    success &= arm_group.go(wait=True)
    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group
    print(self.arm_group.get_planning_frame)
    # Get the current pose and display it
    pose = arm_group.get_current_pose("tool_frame")
    #rospy.loginfo("Actual cartesian pose is : ")
    #rospy.loginfo(pose.pose)
    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose, end_effector_link="tool_frame")

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

def main():
  example = GraspDatasetTrajectories()
  rospy.spin()

if __name__ == '__main__':
  main()