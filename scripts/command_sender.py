import xpc.xpc as xpc

import rospy
import xplane_ros.msg as xplane_msgs

import rosflight_msgs.msg as rosflight_msgs

class CommandSender():
    def __init__(self, client):
        #self.client =  xpc.XPlaneConnect()
        self.client = client

        # Listen on the topic for an control commands from the user
        self.controlSub =  rospy.Subscriber("/xplane/my_control", xplane_msgs.Controls, self.controlCallback)

        self.rosplaneControlSub = rospy.Subscriber("/fixedwing/command", rosflight_msgs.Command, self.rosplaneControlCallback)

    
    def poseCallback(self, msg):
        if msg is not None:
            print(msg.el)
    
    def controlCallback(self, msg):
        # send the user's control commands to XPlane 
        if msg is not None:
            ctrl = [msg.elevator, msg.aileron, msg.rudder, msg.throttle]
            self.client.sendCTRL(ctrl)
        
    def rosplaneControlCallback(self, msg):
        if msg is not None:
            ctrl = [msg.y,msg.x, msg.z, msg.F]
            self.client.sendCTRL(ctrl)
        