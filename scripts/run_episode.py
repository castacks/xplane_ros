#!/usr/bin/env python

# XPC
import xpc.xpc as xpc 
from state_reader import StateReader
from command_sender import CommandSender
from traffic_sender import TrafficSender
from xplane_ros_wrapper import XPlaneRosWrapper
#ROS
import rospy 
import numpy as np








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


        rospy.spin() #!/usr/bin/env python3


