#! /usr/bin/env python


### Next steps
# Try using the error in position to set the velocity

import rospy

import actionlib
import revised_trajectory.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectory
from kinova_msgs.msg import JointVelocity as KinovaVelocity
from kinova_msgs.msg import JointAngles
from copy import deepcopy
from math import pi
from numpy import sign, all, full, isclose



import actionlib_tutorials.msg

class JointTrajectoryAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self, name):
        # Start the action server
        self._action_name = "/j2s7s300/follow_joint_trajectory"
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        # Create the velocity publisher
        self.velocity_pub = rospy.Publisher('/j2s7s300_driver/in/joint_velocity', KinovaVelocity, queue_size=2)
        # Create the joint state subscriber
        self.state_subscriber = rospy.Subscriber('/j2s7s300_driver/out/joint_angles', JointAngles, self.state_cb)
        

        for i in range(10):
            rospy.loginfo("New AC setup!")

        # Create variable to store our JointTrajectory
        self.trajectory = JointTrajectory()

        self.parsed_trajectory = [] # in degrees/s
        self.parsed_positions = [] # in degrees
        self.trajectory_time = []
        self.active_state = False

        self.current_state = []
        self.last_state_update_time = rospy.Time.now()

        self.index = 1
        self.goal_contsraint = .01 # Tolerance from the goal allowed in degrees


      
    def execute_cb(self, goal):
        """ This function executes whenever a joint trajectory is sent from MoveIt!
        """
        # Save our trajectory
        self.trajectory = goal
        # Parse the trajectory
        self.parse_trajectory(goal)
        rospy.loginfo("Successfully parsed goal")
        rospy.loginfo(self.current_state)

        rate = rospy.Rate(50)
        for i in range(len(self.trajectory_time)-1):
            
            joint_status = full((7, 1), False)
            # Get the direction of motion for error checking
            direction = sign([self.parsed_trajectory[self.index].joint1, 
                                self.parsed_trajectory[self.index].joint2, 
                                self.parsed_trajectory[self.index].joint3, 
                                self.parsed_trajectory[self.index].joint4, 
                                self.parsed_trajectory[self.index].joint5, 
                                self.parsed_trajectory[self.index].joint6, 
                                self.parsed_trajectory[self.index].joint7])
            
            # Set joint velocities, update if are are within goal constraints
            velocity_msg = self.parsed_trajectory[self.index]
            # Theoretically loop through sending joint velocity and the callback here
            max_state_update_time_before_cancel = .25

            while True:
                # Check that we received updated states recently 
                if rospy.Time.now() - self.last_state_update_time > rospy.Duration(max_state_update_time_before_cancel):
                    # Break and stop arm and report failure
                    print("Time error!")
                    break
                
                # Check state error
                error = self.state_error()
                

                

                print("test")
                print(direction)
                print("traj")
                print(self.parsed_trajectory[self.index])


                # For joint1
                if (isclose(direction[0], 1.0) and error[0] < self.goal_contsraint) or (isclose(direction[0], -1.0) and error[0] > -self.goal_contsraint):
                    # Joint 1 within range
                    velocity_msg.joint1 = 0.0 
                    joint_status[0] = True
                    rospy.loginfo("Joint 1 in range")
                if (isclose(direction[1], 1.0) and error[1] < self.goal_contsraint) or (isclose(direction[1], -1.0) and error[1] > -self.goal_contsraint):
                    # Joint 1 within range
                    velocity_msg.joint2 = 0.0 
                    joint_status[1] = True
                    rospy.loginfo("Joint 2 in range")
                if (isclose(direction[2], 1.0) and error[2] < self.goal_contsraint) or (isclose(direction[2], -1.0) and error[2] > -self.goal_contsraint):
                    # Joint 1 within range
                    velocity_msg.joint3 = 0.0
                    joint_status[2] = True
                    rospy.loginfo("Joint 3 in range")
                if (isclose(direction[3], 1.0) and error[3] < self.goal_contsraint) or (isclose(direction[3], -1.0) and error[3] > -self.goal_contsraint):
                    # Joint 1 within range
                    velocity_msg.joint4 = 0.0 
                    joint_status[3] = True
                    rospy.loginfo("Joint 4 in range")
                if (isclose(direction[4], 1.0) and error[4] < self.goal_contsraint) or (isclose(direction[4], -1.0) and error[4] > -self.goal_contsraint):
                    # Joint 1 within range
                    velocity_msg.joint5 = 0.0 
                    joint_status[4] = True 
                    rospy.loginfo("Joint 5 in range")
                if (isclose(direction[5], 1.0) and error[5] < self.goal_contsraint) or (isclose(direction[5], -1.0) and error[5] > -self.goal_contsraint):
                    # Joint 1 within range
                    velocity_msg.joint6 = 0.0 
                    joint_status[5] = True
                    rospy.loginfo("Joint 6 in range")
                if (isclose(direction[6], 1.0) and error[6] < self.goal_contsraint) or (isclose(direction[6], -1.0) and error[6] > -self.goal_contsraint):
                    # Joint 1 within range
                    velocity_msg.joint7 = 0.0 
                    joint_status[6] = True
                    rospy.loginfo("Joint 7 in range")
                
                print(joint_status)
                all_zeros = all(joint_status)
                print("all_zeros: ", all_zeros)
                self.velocity_pub.publish(velocity_msg)
                if all_zeros:
                    print("All zeros!")
                    rospy.sleep(2.0)
                    self.index += 1
                    break
                
                # Wait a small period of time before sending next joint velocity command
                rate.sleep()
            


        test = raw_input("hit enter")

        # helper variables
        r = rospy.Rate(1)
        success = True
        print(goal)
        
        rospy.loginfo("Callback happened!")
        
        # append the seeds for the fibonacci sequence
        #self._feedback.sequence = []
        ##self._feedback.sequence.append(0)
        #self._feedback.sequence.append(1)
        
       
        # start executing the action
        # check that preempt has not been requested by the client
        #if self._as.is_preempt_requested():
        #    rospy.loginfo('%s: Preempted' % self._action_name)
        #    self._as.set_preempted()
        #    success = False
        #    break
        #self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
        #self._as.publish_feedback(self._feedback)
        # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #r.sleep()
        
        #if success:
        #    self._result.sequence = self._feedback.sequence
        ##    rospy.loginfo('%s: Succeeded' % self._action_name)
        #    self._as.set_succeeded(self._result)

    def parse_trajectory(self, trajectory):
        # Clear the previous parsed trajectory
        self.index = 1
        self.parsed_trajectory = []
        self.parsed_positions = []
        self.trajectory_time = []

        num_poses = len(trajectory.trajectory.points)
        #rospy.loginfo(len(trajectory.trajectory.points))
        # Loop through the points and add them to our list. Also convert to degrees
        for i in range(num_poses):
            velocity_msg = KinovaVelocity()
            velocity_msg.joint1 = trajectory.trajectory.points[i].velocities[0] * 180/pi*4
            velocity_msg.joint2 = trajectory.trajectory.points[i].velocities[1] * 180/pi*4
            velocity_msg.joint3 = trajectory.trajectory.points[i].velocities[2] * 180/pi*4
            velocity_msg.joint4 = trajectory.trajectory.points[i].velocities[3] * 180/pi*4
            velocity_msg.joint5 = trajectory.trajectory.points[i].velocities[4] * 180/pi*4
            velocity_msg.joint6 = trajectory.trajectory.points[i].velocities[5] * 180/pi*4
            velocity_msg.joint7 = trajectory.trajectory.points[i].velocities[6] * 180/pi*4

            positions = [trajectory.trajectory.points[i].positions[0] * 180/pi,
                        trajectory.trajectory.points[i].positions[1] * 180/pi,
                        trajectory.trajectory.points[i].positions[2] * 180/pi,
                        trajectory.trajectory.points[i].positions[3] * 180/pi,
                        trajectory.trajectory.points[i].positions[4] * 180/pi,
                        trajectory.trajectory.points[i].positions[5] * 180/pi,
                        trajectory.trajectory.points[i].positions[6] * 180/pi]
            
            self.parsed_positions.append(deepcopy(positions))
            self.parsed_trajectory.append(deepcopy(velocity_msg))
            self.trajectory_time.append(trajectory.trajectory.points[i].time_from_start.nsecs)


        #print(self.parsed_trajectory)
        #print("First", self.parsed_trajectory[0])
        #print(trajectory)

        """#ospy.loginfo(trajectory)
        us = raw_input("enter to continue")

        rate = rospy.Rate(100)
        velocity_msg = KinovaVelocity()
        velocity_msg.joint1 = 3
        velocity_msg.joint2 = 3
        velocity_msg.joint3 = 3
        velocity_msg.joint4 = 3
        velocity_msg.joint5 = 3
        velocity_msg.joint6 = 3
        velocity_msg.joint7 = 3

        #for i in range(100):
        #    # Loop through the 7 joints
        #    self.velocity_pub.publish(velocity_msg)
        #    rospy.loginfo("Sent vel command")
        #    rate.sleep()"""

    def state_error(self):
        # Compares the target state to the current state and returns the error
        print("jt1 targ: ", self.parsed_positions[self.index][0])
        print("jt1 current: ", self.current_state.joint1)
        error_out = [self.parsed_positions[self.index][0] - self.current_state.joint1,
                     self.parsed_positions[self.index][1] - self.current_state.joint2,
                     self.parsed_positions[self.index][2] - self.current_state.joint3,
                     self.parsed_positions[self.index][3] - self.current_state.joint4,
                     self.parsed_positions[self.index][4] - self.current_state.joint5,
                     self.parsed_positions[self.index][5] - self.current_state.joint6,
                     self.parsed_positions[self.index][6] - self.current_state.joint7]
        print("Pos error", error_out)
        return error_out
    
    def state_cb(self, state):
        # Update the class state variable and last time
        #rospy.loginfo(state)
        self.current_state = state
        self.last_state_update_time = rospy.Time.now()
        
        
            




        
if __name__ == '__main__':
    rospy.init_node('j2s7s300_joint_trajectory_action_server')
    server = JointTrajectoryAction(rospy.get_name())
    rospy.spin()