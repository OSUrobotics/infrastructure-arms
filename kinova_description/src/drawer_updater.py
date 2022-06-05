#!/usr/bin/env python
import rospy, sys, rosbag, csv
from math import pow
from sensor_msgs.msg import JointState
#from infrastructure_msgs.msg import DoorSensors
from kinova_description.srv import Data, DataResponse

"""
publisher that publishes to /drawer_distance to control joint pose of drawer robot model.
drawer_data service not being used in current implementation
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
        # Print the message
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
        # Print the message
        print(msg.trial_number)
        print(msg.collection_end_time)
    timestamp_bag.close()

#returns drawer distance currently
# def replay_callback(request):
#     rospy.loginfo("callback for service ran!")
#     global distances, fps
#     return DataResponse(distances, fps)

"""
params: csv file containing drawer data
"""
if __name__ == "__main__":
    rospy.init_node('drawer_updater', argv=sys.argv)
    pub = rospy.Publisher('drawer_distance', JointState, queue_size=10)
    
    # parseRosBags(sys.argv[1], sys.argv[2])
    sensor_bag = rosbag.Bag(sys.argv[1])

    # fps, distances = getFPS(sys.argv[1])
    
    #specifically wait until distances and fps are retrieved
    # replay_service = rospy.Service('drawer_data', Data, replay_callback)
    # rate = rospy.Rate(fps)
    rate = rospy.Rate(32.5) # avg rate of data publishing in infrastructure system for drawer currently.
    while not rospy.is_shutdown():
        # for i in distances:
        for topic, msg, t in sensor_bag.read_messages(None):
            message = JointState()
            message.name = ["drawer_slider"]
            message.position = [msg.tof]
            message.velocity = []
            message.effort = []
            # if(msg.current_time )
            pub.publish(message)
            rate.sleep()
