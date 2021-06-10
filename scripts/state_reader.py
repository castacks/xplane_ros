import xpc

import rospy
import xplane_ros.msg as xplane_msgs

# Class to extract position and controls related information from XPlane 
class StateReader:
    def __init__(self):
        # instantiate connection to XPC
        self.client =  xpc.XPlaneConnect()


        self.posePub = rospy.Publisher("/xplane/sim/flightmodel/odom", xplane_msgs.Position, queue_size = 10)
        self.controlPub = rospy.Publisher("/xplane/sim/flightmodel/controls", xplane_msgs.Controls, queue_size = 10)


    def sensor_update(self):
        # get global position information from XPlane
        pos = self.client.getPOSI()
        # convert data to ros msgs
        msg = xplane_msgs.Position()
        msg.lat = pos[0]
        msg.lon = pos[1]
        msg.el = pos[2]
        msg.roll = pos[3]
        msg.pitch = pos[4]
        msg.yaw = pos[5]
        msg.gear = pos[6]

        self.posePub.publish(msg)
    
    def control_update(self):
        # get control surfaces information from XPlane
        ctrl = self.client.getCTRL()
        # convert data to ros msgs 
        msg = xplane_msgs.Controls()
        msg.elevator = ctrl[0]
        msg.aileron = ctrl[1]
        msg.rudder = ctrl[2]
        msg.throttle = ctrl[3]
        msg.gear = ctrl[4]
        msg.flaps = ctrl[5]
        msg.speed_brakes = ctrl[6]

        self.controlPub.publish(msg)
    
    # def velocity_update(self):
    #     drefs = []


