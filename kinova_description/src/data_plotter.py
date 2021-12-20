#!/usr/bin/env python

import rospy, sys
import csv
import numpy as np
import pyqtgraph as pg
from sensor_msgs.msg import JointState
from math import pow
from pyqtgraph.Qt import QtGui, QtCore
from kinova_description.srv import Data

# look at p6 (uses Qtimer and timeout.connect(update_fxn) to update which is same as Control_plane_window.py)
# multiple ways to go about plotting data:
# - subscribe to topic and read sensor values, then plot under while not rospy.is_shutdown() loop (needs to be seperate than drawer_distance for FSRs. also need to get sleep rate)
# - use Qtimer with timeout update function that updates graph and just rospy.spin (needs sleep rate for Qtimer and sensor data beforehand (action/service call or param))
# - make action/service call to main node (drawer_updater.py currently) that retrieves all data information including sleep rate, then plot under while loop with rospy.sleep()

# Currently uses Qtimer with making service call to drawer_updater to get data and fps. Runs ~1 second behind replay (but runs in real-time!)

# attempt for multithreading (run() will be main loop for QtWidget)
# unstable. Freezes if plot is moved at all. can be fixed?
class PlotUpdater(QtCore.QRunnable):
    def __init__(self):#, update_time):
        QtCore.QRunnable.__init__(self)
        # self.fps = update_time
        # self.data = []

    def run(self):
        # subscriber. use the hardware_infsensor topic for live data
        subscriber = rospy.Subscriber('drawer_distance', JointState, callback)
        rospy.spin()
        # while True:
            # QtCore.QMetaObject.invokeMethod(self.it, "set_data",
            #                          QtCore.Qt.QueuedConnection,
            #                          QtCore.Q_ARG(list, self.data))
            # QtCore.QThread.msleep(self.fps)

# unstable. Freezes if plot is moved at all. can be fixed?
def callback(msg):
    global curve, live_data
    live_data.append(msg.position[0])
    curve.setData(live_data)

# function that gets called everytime Qtimer times out (current implementation, doesn't freeze)
def update():
    # global curve, all_data, ptr
    # # replay via csv
    # if not ptr == len(all_data.data):
    #     curve.setData(all_data.data[0:ptr])
    #     ptr += 1
    global curve, get_live_data
    curve.setData(get_live_data().data)

if __name__ == "__main__":
    rospy.init_node('data_plotter', argv=sys.argv)

    # basic Qt setup
    app = QtGui.QApplication([])
    win = pg.GraphicsLayoutWidget(show=True, title="Drawer Distance")
    win.resize(1000,600)
    win.setWindowTitle('distance plot')

    # Enable antialiasing for prettier plots and set plot features
    pg.setConfigOptions(antialias=True)
    p6 = win.addPlot(title="values")
    p6.setLabel('left', "Drawer distance", units='m')
    p6.setLabel('bottom', "Frames", units='')
    curve = p6.plot(pen='y')

    # --- replay via csv --- 

    # # use service call
    # rospy.wait_for_service('drawer_data')
    # get_data = rospy.ServiceProxy('drawer_data', Data)
    # all_data = get_data()

    # # plot first point and set range
    # p6.setXRange(0, len(all_data.data), padding=0)
    # p6.setYRange(0.15, -0.15, padding=0)
    # curve.setData(all_data.data[0:0])
    # p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    # ptr = 1

    # # setup timer
    # timer = QtCore.QTimer()
    # # every time timer finishes, it executes update() function
    # timer.timeout.connect(update)
    # timer.start(all_data.fps) #in ms

    # --- replay via subscriber ---
    
    rospy.wait_for_service('live_data')
    get_live_data = rospy.ServiceProxy('live_data', Data)
    curve.setData([])
    #p6.setXRange(0, 3000, padding=0) #hard, change!
    p6.setYRange(0.15, -0.15, padding=0)
    #p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted

    timer = QtCore.QTimer()
    # every time timer finishes, it executes update() function
    timer.timeout.connect(update)
    timer.start(32) #in ms

    # attempt runnable thread
    # runnable = PlotUpdater()
    # QtCore.QThreadPool.globalInstance().start(runnable)
    # subscriber = rospy.Subscriber('drawer_distance', JointState, callback)

    #once called, will hang on this function. Can use sys.exit(app.exec_())
    #sys.exit(QtGui.QApplication.instance().exec_())
    QtGui.QApplication.instance().exec_() #keeps node up this way

    # rate = rospy.Rate(30) #hz
    # while not rospy.is_shutdown():
    #     rospy.loginfo("ran loop")
    #     curve.setData(data[ptr%10])
    #     if ptr == 0:
    #         p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    #     ptr += 1
    #     rate.sleep()