#!/usr/bin/env python

import rospy 
import xplane_ros.msg as xplane_msgs
from nav_msgs.msg import Odometry

class Takeoff:
    def __init__(self):
        self.HEIGHT = -1
        self.poseSub =  rospy.Subscriber("/xplane/sim/flightmodel/odom", Odometry, self.poseCallback)
        self.controlsPub = rospy.Publisher("/xplane/my_control", xplane_msgs.Controls, queue_size=10)

        self.pose = Odometry()

        self.ctrl = xplane_msgs.Controls()
        self.ctrl.aileron = -998
        self.ctrl.rudder = -998
        self.ctrl.throttle = -998
        self.ctrl.elevator =-998

    def poseCallback(self, msg):
        if self.HEIGHT == - 1:
            self.HEIGHT = msg.el + 300
        self.pose = msg
        print("In Takeoff cb :", self.pose.lat)
    
    def takeoff(self):
        self.ctrl.elevator = 0.0
        for i in range(100):

            if self.ctrl.throttle == -998:
                self.ctrl.throttle = 0.0
            self.ctrl.throttle = min(1.0, self.ctrl.throttle + 0.01)
            if (i>50): 
                self.ctrl.elevator = min(0.6, self.ctrl.elevator + 0.1)
            self.controlsPub.publish(self.ctrl)
            print("In takeoff()", i)
            rospy.sleep(0.1)
        

if __name__ == '__main__':

    rospy.init_node("takeoff_pilot", anonymous=True)
    takeoff = Takeoff()
    takeoff.takeoff()
    rospy.spin()

        

    

    
