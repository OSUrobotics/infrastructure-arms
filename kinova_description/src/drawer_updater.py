#!/usr/bin/env python
import rospy, sys, rosbag, csv
from math import pow
from sensor_msgs.msg import JointState
#from infrastructure_msgs.msg import DoorSensors
from kinova_description.srv import Data, DataResponse

"""
publisher that publishes to /drawer_distance to control joint pose of drawer robot model.
 
 - Currently does not utilize time stamps and publishes all trials in sensor rosbag.
 
 - Once it reaches last trial in bag file, will loop back to trial 1.

 - Publishes at same frequency as rosbag file.

 - Skips down time between trials.

 - Assumes that if there is > 1 second time skip between msgs, then new trial has started.

 - Uses -.001 as a multiplier for tof values to scale down to Rviz units as well as correct direction.

 Author: Ryan Roberts
 Email : roberyan@oregonstate.edu
 Date  : 5/2022
"""

def getFPS(data):
    time_data = []
    distance_data = []
    with open(data, mode='r') as f:
        reader = csv.reader(f, delimiter=' ', quotechar='|')
        for row in reader:
            time_data.append(float(row[-1]))
            tmp_dist = float(row[0].replace(",", ""))
            #rospy.loginfo(tmp_dist)
            if tmp_dist < 0:
                distance_data.append(0)
            else:
                distance_data.append(tmp_dist* -.001) #reverse sign for model in rviz
    total_time = (time_data[len(time_data)-1] - time_data[0]) / pow(10,9)
    return (len(time_data) / total_time), distance_data

def parseRosBags(sensors, timestamps):
    topics = None # specifying topic doesnt seem to work

    sensor_bag = rosbag.Bag(sensors)
    # know msg type is DoorSensors
    for topic, msg, t in sensor_bag.read_messages(topics):
        print("\n# =================== Sensor Bag ====================")
        print("# timestamp (sec): {:.9f}".format(t.to_sec())),
        print("# - - -")
        print(msg.tof)
        print(msg.current_time)
        break
    sensor_bag.close()

    timestamp_bag = rosbag.Bag(timestamps)
    # know msg type is DataTimestamps
    for topic, msg, t in timestamp_bag.read_messages(topics):
        print("\n# =================== Timestamp Bag ====================")
        print("# timestamp (sec): {:.9f}".format(t.to_sec())),
        print("# - - -")
        print(msg.trial_number)
        print(msg.total_time)
    timestamp_bag.close()

"""
params: 
    1 - Rosbag file for drawer sensor data (hardware_infsensor)
    2 - Rosbag file for timestamps (hardware_timestamps)
"""
if __name__ == "__main__":
    rospy.init_node('drawer_updater', argv=sys.argv)
    drawer_pub = rospy.Publisher('drawer_distance', JointState, queue_size=10)
    
    sensor_bag = rosbag.Bag(rospy.get_param("/sensor_data_rosbag"))
    robot_pub = rospy.Publisher(rospy.get_param("/robot_topic_name","/"), JointState, queue_size=10)

    # fps, distances = getFPS(sys.argv[1])
    
    # rate = rospy.Rate(32.5) # avg rate of data publishing in infrastructure system for drawer currently.

    while not rospy.is_shutdown():
        # TODO: implement wait until pyqt service is up and running
        # repeat trial indefinitely
        start = rospy.Time.now()
        sim_start = None
        # for i in distances:
        for topic, msg, t in sensor_bag.read_messages(None):
            now = rospy.Time.now()
            if sim_start is None:
                sim_start = t
            else:
                real_time = now - start
                sim_time = t - sim_start
                if sim_time > real_time:
                    sleep_time = sim_time - real_time
                    # assuming if time difference is > 1 second, then trial is over & we skip to next one - not needed with new data structure
                    if sleep_time.secs <= 1:
                        rospy.sleep(sleep_time)
                    else:
                        # reset real time and start next trial
                        start = rospy.Time.now()
                        sim_start = None
            if (topic == "/hardware_infsensor"):
                message = JointState()
                message.name = ["drawer_slider"]
                message.position = [msg.tof * -.001] #reverse sign for model in rviz
                message.velocity = []
                message.effort = []
                drawer_pub.publish(message)
            else:
                # robot joint state topic. msg should already be a joint state. TODO: add error checking for msg type
                robot_pub.publish(msg)
                
            if rospy.is_shutdown():
                break
        # break # break if want to stop after 1 run
    
    # before process ends:
    sensor_bag.close()
