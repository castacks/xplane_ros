#!/usr/bin/env python
import rospy 


import rosplane_msgs.msg as rosplane_msgs
import xplane_ros.msg as xplane_msgs
import numpy as np
import xpc.xpc as xpc
from time import sleep

class Options:
    def __init__(self):
        self.hold_roll = False
        self.hold_pitch = True
        self.roll_step = (np.pi/180.0) * (15)
        self.pitch_step = (np.pi/180.0) * (0)


class RosplaneTuner:
    def __init__(self, client):
        self.options = Options()
        self.tuner_commands = rosplane_msgs.Tuner_Commands()

        '''Parameters present in Control_Commands; not needed for now'''
        self.tuner_commands.Va_c = 60
        self.tuner_commands.h_c = 50
        self.tuner_commands.chi_c = 0
        self.tuner_commands.phi_ff = 0

        '''Parameters for tuning roll and pitch'''
        self.tuner_commands.hold_roll = self.options.hold_roll
        self.tuner_commands.hold_pitch = self.options.hold_pitch
        '''Default commanded roll and pitch values are 0.0 rad'''
        self.tuner_commands.phi_c = 0.0
        self.tuner_commands.theta_c = 0.0

        self.tunerPub = rospy.Publisher("/fixedwing/tuner_commands", rosplane_msgs.Tuner_Commands, queue_size=10)
        rospy.Timer(period=rospy.Duration(0.1), callback=self.command_update)

        self.count = 0
        #       Lat     Lon   Alt   Pitch  Roll  Yaw Gear
        posi = [-998, -998, 1000,     1.0,    0.0,   0.0,  1]
        client.sendPOSI(posi)

        drefs = []
        # drefs.append("sim/flightmodel/position/indicated_airspeed")
        drefs.append("sim/flightmodel/position/local_vx")
        drefs.append("sim/flightmodel/position/local_vy")
        drefs.append("sim/flightmodel/position/local_vz")
        values = [0,-2,-50]
        # values = [60]
        client.sendDREFs(drefs, values)

        ctrl = [0.0, 0.0, 0.0, 0.8]
        client.sendCTRL(ctrl)

        # client.pauseSim(True)
        # sleep()
        # client.pauseSim(False)

    
    def command_update(self, event=None):
        self.count += 1
        '''Wait for 1 sec before sending step input'''
        if self.count > 30:

            if self.tuner_commands.hold_roll:
                self.tuner_commands.phi_c = self.options.roll_step

            elif self.tuner_commands.hold_pitch:
                self.tuner_commands.theta_c = self.options.pitch_step
            
        self.tunerPub.publish(self.tuner_commands)


if __name__ == '__main__':  
    rospy.init_node('xplane_tuner', anonymous=True)
    with xpc.XPlaneConnect(timeout=10000) as client:
        rosplaneTuner = RosplaneTuner(client)
        rospy.spin()