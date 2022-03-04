#!/usr/bin/env python
import rospy, sys
import csv
from sensor_msgs.msg import JointState
from math import pow
from kinova_description.srv import Data, DataResponse

#publisher that publishes to /drawer_distance to control joint pose of drawer robot model
# drawer_data service not being used in current implementation

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

#returns drawer distance currently
def replay_callback(request):
    rospy.loginfo("callback for service ran!")
    global distances, fps
    return DataResponse(distances, fps)

"""
params: csv file containing drawer data
"""
if __name__ == "__main__":
    rospy.init_node('drawer_updater', argv=sys.argv)
    pub = rospy.Publisher('drawer_distance', JointState, queue_size=10)
    # for i in sys.argv:
    #     rospy.loginfo(i)
    fps, distances = getFPS(sys.argv[1])
    #specifically wait until distances and fps are retrieved
    replay_service = rospy.Service('drawer_data', Data, replay_callback)
    #fps, distances = getFPS(rospy.get_param('~drawer_data'))
    #print(distances)
    #print(fps)
    rate = rospy.Rate(fps)
    while not rospy.is_shutdown():
        for i in distances:
            message = JointState()
            message.name = ["drawer_slider"]
            message.position = [i]
            message.velocity = []
            message.effort = []
            pub.publish(message)
            rate.sleep()
