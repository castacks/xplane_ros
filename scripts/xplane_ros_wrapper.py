#!/usr/bin/env python

# XPC
import xpc.xpc as xpc 
from state_reader import StateReader
from command_sender import CommandSender
from traffic_sender import TrafficSender

#ROS
import rospy 
import numpy as np


class XPlaneRosWrapper:
    def __init__(self, stateReader, commandSender,trafficSender):
        self.stateReader = stateReader
        self.commandSender = commandSender
        self.trafficSender = trafficSender

        '''single function for all the information updates to avoid the issue of synchronization'''
        rospy.Timer(period=rospy.Duration(0.1), callback=self.sensor_update)
    
    def sensor_update(self, step):
        '''Extract all the useful information sequentially here'''

        # if self.trafficSender is not None:
            # self.trafficSender.sendTraffic()


        if self.stateReader is not None:
            # self.stateReader.sensor_update()
            # self.stateReader.control_update()
            self.stateReader.sensor_update2()


    def reset(self,client,posi,groundspeed = 50):
        
        psi = posi[5]
        print("PSI:",psi)
        client.sendPOSI(posi, 0)
        fwd_vel_z = -groundspeed*np.cos(np.deg2rad(psi)) 
        fwd_vel_x = groundspeed*np.sin(np.deg2rad(psi)) 

        client.sendDREF("sim/flightmodel/position/local_vx", fwd_vel_x)
        client.sendDREF("sim/flightmodel/position/local_vz", fwd_vel_z)
        client.sendDREF("sim/flightmodel/position/local_vy", 0)

        client.sendDREF("sim/flightmodel/position/local_ax", 0)
        client.sendDREF("sim/flightmodel/position/local_az", 0)
        client.sendDREF("sim/flightmodel/position/local_ay", 0)

        client.sendDREF("sim/flightmodel/position/beta", 0)
        client.sendDREF("sim/flightmodel/position/alpha", 0)

        client.sendDREF("sim/flightmodel/position/M", 0)
        client.sendDREF("sim/flightmodel/position/N", 0)
        client.sendDREF("sim/flightmodel/position/L", 0)
        client.sendDREF("sim/flightmodel/position/P", 0)
        client.sendDREF("sim/flightmodel/position/Q", 0)
        client.sendDREF("sim/flightmodel/position/R", 0)

        client.sendDREF("sim/flightmodel/position/Prad", 0)
        client.sendDREF("sim/flightmodel/position/Qrad", 0)
        client.sendDREF("sim/flightmodel/position/Rrad", 0)
        client.sendDREF("sim/flightmodel/position/P_dot", 0)
        client.sendDREF("sim/flightmodel/position/Q_dot", 0)
        client.sendDREF("sim/flightmodel/position/R_dot", 0)

    # def read_episode(self,path,episode_num = 1):




    

if __name__ == '__main__':
    # start the interface node
    rospy.init_node('xplane_ros_wrapper', anonymous=True)
    with xpc.XPlaneConnect(timeout=20000) as client:
        ''' instantiate reader and sender objects '''
        

        stateReader = StateReader(client)
        commandSender = CommandSender(client)
        trafficSender = TrafficSender(client)


        
        '''instantiate wrapper object'''
        xplaneRosWrapper = XPlaneRosWrapper(stateReader, commandSender, trafficSender)
        posi = [40.774548,-79.959237, 400, 0,    0,  70,  1]
        xplaneRosWrapper.reset(client,posi)
        rospy.spin() #!/usr/bin/env python3


