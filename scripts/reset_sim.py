import xpc.xpc as xpc

import rospy
import xplane_ros.srv as xplane_srv

import numpy as np

class ResetSim():
    def __init__(self, client):

        self.client = client
        
        self.reset_sim = rospy.Service('reset_sim', xplane_srv.ResetSim, self.handle_reset)

    def handle_reset(self,req):
        
        posi = [req.lat,req.lon,req.alt,0,0,req.yaw,1]
        return self.reset(posi=posi,groundspeed=req.groundspeed)


    def reset(self,posi,groundspeed = 50):
        
        psi = posi[5]
        self.client.sendPOSI(posi, 0)
        fwd_vel_z = -groundspeed*np.cos(np.deg2rad(psi)) 
        fwd_vel_x = groundspeed*np.sin(np.deg2rad(psi)) 

        self.client.sendDREF("sim/flightmodel/position/local_vx", fwd_vel_x)
        self.client.sendDREF("sim/flightmodel/position/local_vz", fwd_vel_z)
        self.client.sendDREF("sim/flightmodel/position/local_vy", 0)
        
        self.client.sendDREF("sim/flightmodel/position/local_ax", 0)
        self.client.sendDREF("sim/flightmodel/position/local_az", 0)
        self.client.sendDREF("sim/flightmodel/position/local_ay", 0)
        
        self.client.sendDREF("sim/flightmodel/position/theta", 0)
        self.client.sendDREF("sim/flightmodel/position/phi", 0)
                
        self.client.sendDREF("sim/flightmodel/position/P", 0)
        self.client.sendDREF("sim/flightmodel/position/Q", 0)
        self.client.sendDREF("sim/flightmodel/position/R", 0)
        
        self.client.sendDREF("sim/flightmodel/position/Prad", 0)
        self.client.sendDREF("sim/flightmodel/position/Qrad", 0)
        self.client.sendDREF("sim/flightmodel/position/Rrad", 0)
        
        self.client.sendDREF("sim/flightmodel/position/P_dot", 0)
        self.client.sendDREF("sim/flightmodel/position/Q_dot", 0)
        self.client.sendDREF("sim/flightmodel/position/R_dot", 0)

        return True