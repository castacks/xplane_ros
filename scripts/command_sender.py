#!/usr/bin/env python


import xpc.xpc as xpc

import rospy
import xplane_ros.msg as xplane_msgs

import rosflight_msgs.msg as rosflight_msgs

class CommandSender():
    def __init__(self, client):
        #self.client =  xpc.XPlaneConnect()
        self.client = client
        '''Turn off brake'''
        self.sendBrake(0)
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
        # posi = [40.77465057373047,-79.95907592773438, 381.69171142578125, 0,    0,  90,  1]
        # self.client.sendPOSI(posi, 1)
        if msg is not None:

            if msg.x < -1.0 or msg.x > 1.0:
                msg.x = -998.0
            if msg.y < -1.0 or msg.y > 1.0:
                msg.y = -998.0
            if msg.z < -1.0 or msg.z > 1.0:
                msg.z = -998.0
            if msg.F < 0.0 or msg.F > 1.0:
                msg.F = -998.0
            ctrl = [msg.y,msg.x, msg.z, msg.F]
            self.client.sendCTRL(ctrl)
    
    def sendBrake(self, brake):
        '''Set the parking brake to on or off
            brake: 0 - OFF; 1 - ON
        '''
        # print("Brakes Off!!")
        self.client.sendDREF("sim/flightmodel/controls/parkbrake", brake)
        

if __name__ == '__main__':
    # start the interface node
    rospy.init_node('xplane_ros_command_wrapper', anonymous=True)
    with xpc.XPlaneConnect(timeout=20000) as client:
        
        '''instantiate wrapper object'''

        # trafficSender = TrafficSender(client)
        commandSender = CommandSender(client)

        rospy.spin() #!/usr/bin/env python3