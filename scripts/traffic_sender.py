import xpc.xpc as xpc

import rospy
import xplane_ros.msg as xplane_msgs

import rosflight_msgs.msg as rosflight_msgs

class TrafficSender():
    def __init__(self, client):
        #self.client =  xpc.XPlaneConnect()
        self.client = client
        # Listen on the topic for an control commands from the user
        
    
    def sendTraffic(self):
        print("Sending Traffic!!")


        # self.client.sendDREF("sim/multiplayer/position/plane2_lat",40.77465057373047)
        # self.client.sendDREF("sim/multiplayer/position/plane2_lon", -79.95907592773438)
        # self.client.sendDREF("sim/multiplayer/position/plane2_el", 381.69171142578125)

        # self.client.sendDREF("sim/multiplayer/position/plane2_the",0)
        # self.client.sendDREF("sim/multiplayer/position/plane2_phi",0)
        # self.client.sendDREF("sim/multiplayer/position/plane2_psi",0)

        # self.client.sendDREF("sim/multiplayer/position/plane1_v_x",10)
        # self.client.sendDREF("sim/multiplayer/position/plane2_v_y",0)
        # self.client.sendDREF("sim/multiplayer/position/plane2_v_z",0)

        # posi = [40.77465057373047,-79.95907592773438, 381.69171142578125, 0,    0,   0,  1]
        # self.client.sendPOSI(posi, 1)


        drefs = []
        drefs.append("sim/multiplayer/position/plane1_x")
        drefs.append("sim/multiplayer/position/plane1_y")
        drefs.append("sim/multiplayer/position/plane1_z")

        data = self.client.getDREFs(drefs)
        print(data)
