#!/usr/bin/env python

# XPC
import xpc 
from state_reader import StateReader
from command_sender import CommandSender

#ROS
import rospy 


class XPlaneRosWrapper:
    def __init__(self, stateReader, commandSender):
        self.stateReader = stateReader
        self.commandSender = commandSender

        #rospy.Timer(period=rospy.Duration(0.1), callback=self.position_update)
        # single function for all the information updates to avoid the issue of synchronization
        rospy.Timer(period=rospy.Duration(0.1), callback=self.sensor_update)
    
    def sensor_update(self, step):
        # Extract all the useful information sequentially here
        if self.stateReader is not None:
            self.stateReader.sensor_update()
            self.stateReader.control_update()
    

if __name__ == '__main__':
    # start the interface node
    rospy.init_node('xplane_ros_wrapper', anonymous=True)

    # instantiate reader and sender objects
    stateReader = StateReader()
    commandSender = CommandSender()

    # instantiate wrapper object
    xplaneRosWrapper = XPlaneRosWrapper(stateReader, commandSender)

    rospy.spin()

