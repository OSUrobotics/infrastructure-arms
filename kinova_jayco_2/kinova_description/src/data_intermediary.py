#!/usr/bin/env python
import rospy, sys
from sensor_msgs.msg import JointState
from kinova_description.srv import Data, DataResponse

# subscriber and service that retrieves data from sensor topic and provides service for data_plotter.py
# uses Data service type for ease of use

def live_callback(request):
    global live_data_set
    return DataResponse(live_data_set, -1)

def sub_callback(msg):
    global live_data_set
    live_data_set.append(msg.position[0])


if __name__ == "__main__":
    rospy.init_node('data_intermediary', argv=sys.argv)
    live_data_set = []
    # replace topic with hardware_infsensor once replaying rosbags
    subscriber = rospy.Subscriber('drawer_distance', JointState, sub_callback)
    live_service = rospy.Service('live_data', Data, live_callback)
    rospy.spin()