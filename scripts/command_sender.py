import xpc

import rospy
import xplane_ros.msg as xplane_msgs

class CommandSender():
    def __init__(self):
        self.client =  xpc.XPlaneConnect()

        #self.poseSub =  rospy.Subscriber("/xplane/sim/flightmodel/odom", xplane_msgs.Position, self.poseCallback)
        # Listen on the topic for an control commands from the user
        self.controlSub =  rospy.Subscriber("/xplane/my_control", xplane_msgs.Controls, self.controlCallback)

    
    def poseCallback(self, msg):
        if msg is not None:
            print(msg.el)
    
    def controlCallback(self, msg):
        # send the user's control commands to XPlane 
        ctrl = [msg.elevator, msg.aileron, msg.rudder, msg.throttle]
        if msg is not None:
            self.client.sendCTRL(ctrl)
        