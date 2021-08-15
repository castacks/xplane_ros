#!/usr/bin/env python
import rospy 


import rosplane_msgs.msg as rosplane_msgs
import xplane_ros.msg as xplane_msgs
import numpy as np
import xpc.xpc as xpc
from time import sleep

from dynamic_reconfigure.server import Server
from xplane_ros.cfg import CommandsConfig

'''TRAJ_LIB is true => use rosplane_tuner to run pre recorded commands instead of using it as tuning utility'''
TRAJ_LIB = True 
ACTION_FILE = ""
if TRAJ_LIB:
    ACTIONS_FILE = '/home/rbaijal/ROS_WS/xplane_ros_ws/src/xplane_ros/utils/Actions210_387.txt'
KNOTS_TO_MS = 0.51444444444

# Initial conditions/ commands to give to the control functions
class Options:
    def __init__(self):
        self.hold_roll = False                        # True if you want to invoke roll_hold() in rosplane
        self.hold_pitch = False                       # True if you want to invoke pitch_hold() in rosplane
        self.hold_course = False                      # True if you want to invoke course_hold() in rosplane
        self.roll_step = (np.pi/180.0) * (15)
        self.pitch_step = (np.pi/180.0) * (0)
        self.course_step = (np.pi / 180.0) * (0.0)


class RosplaneTuner:
    def __init__(self, client):
        self.options = Options()
        self.tuner_commands = rosplane_msgs.Tuner_Commands()

        '''Parameters present in Control_Commands; not needed for now'''
        self.tuner_commands.Va_c = 40
        self.tuner_commands.h_c = 800
        self.tuner_commands.chi_c = 0
        self.tuner_commands.phi_ff = 0
        self.tuner_commands.vh_c = 0

        '''Parameters for tuning roll and pitch'''
        self.tuner_commands.hold_roll = self.options.hold_roll
        self.tuner_commands.hold_pitch = self.options.hold_pitch
        '''Default commanded roll and pitch values are 0.0 rad'''
        self.tuner_commands.phi_c = 0.0
        self.tuner_commands.theta_c = 0.0

        if TRAJ_LIB:
            self.file = open(ACTIONS_FILE, 'r')
            self.actions = self.file.readlines()
        # print(self.actions)
        self.count = 0

        '''setup server for dynamic reconfigure'''
        self.srv = Server(CommandsConfig, self.callback)

        # print(lines)

        '''Publish the commanded values for roll, pitch and course'''
        self.tunerPub = rospy.Publisher("/fixedwing/tuner_commands", rosplane_msgs.Tuner_Commands, queue_size=10)
        rospy.Timer(period=rospy.Duration(0.1), callback=self.command_update)


        '''Assuming that the aircraft is currently at the runway, this will spawn the aircraft at a height of 1000 m at the same lat, lon'''
        #       Lat     Lon   Alt   Pitch  Roll  Yaw Gear
        posi = [-998, -998, 1000,     1.0,    0.0,   0.0,  1]
        client.sendPOSI(posi)

        '''Provide initial velocity to the aircraft so that it does not go straight into the ground'''
        drefs = []
        # drefs.append("sim/flightmodel/position/indicated_airspeed")
        drefs.append("sim/flightmodel/position/local_vx")
        drefs.append("sim/flightmodel/position/local_vy")
        drefs.append("sim/flightmodel/position/local_vz")
        values = [0,-2,-40]
        # values = [60]
        client.sendDREFs(drefs, values)

        '''Provide throttle commad to maintain speed before the controllers kick in'''
        ctrl = [0.0, 0.0, 0.0, 0.8]
        client.sendCTRL(ctrl)

        # client.pauseSim(True)
        # sleep()
        # client.pauseSim(False)
    
    def callback(self, config, level):
        '''Setting flags'''
        self.tuner_commands.hold_roll = config.hold_roll
        self.tuner_commands.hold_pitch = config.hold_pitch
        self.tuner_commands.hold_course =  config.hold_course
        self.tuner_commands.hold_altitude = config.hold_altitude
        self.tuner_commands.hold_vh = config.hold_vh
        self.tuner_commands.hold_va = config.hold_va

        '''Commanded values'''
        self.tuner_commands.phi_c = (config.roll_step) # * (np.pi / 180.0)
        self.tuner_commands.theta_c = (config.pitch_step) # * (np.pi / 180.0)
        self.tuner_commands.Va_c = (config.Va_c)
        self.tuner_commands.h_c = config.h_c
        self.tuner_commands.chi_c = (config.chi_c) # * (np.pi/180.0)
        self.tuner_commands.phi_ff = (config.phi_ff) # * (np.pi / 180.0)
        self.tuner_commands.vh_c = config.vh_c

        return config
    
    def command_update(self, event=None):
        if TRAJ_LIB:
            self.count += 1
            index = int(self.count/100.0)
            a = self.actions[0].split()
            if(index < len(self.actions)):
                a = self.actions[index].split()
            print(a)

            self.tuner_commands.hold_roll = True
            self.tuner_commands.hold_vh = True
            self.tuner_commands.hold_va = True

            self.tuner_commands.phi_c = float(a[2])
            self.tuner_commands.Va_c = float(a[0])*KNOTS_TO_MS
            self.tuner_commands.vh_c = float(a[1])
                
        self.tunerPub.publish(self.tuner_commands)


if __name__ == '__main__':  
    rospy.init_node('xplane_tuner', anonymous=True)
    with xpc.XPlaneConnect(timeout=10000) as client:
        rosplaneTuner = RosplaneTuner(client)
        rospy.spin()