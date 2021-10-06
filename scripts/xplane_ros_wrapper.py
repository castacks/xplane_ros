#!/usr/bin/env python

# XPC
import xpc.xpc as xpc 
from state_reader import StateReader
from command_sender import CommandSender
from traffic_sender import TrafficSender
from reset_sim import ResetSim
#ROS
import rospy 


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

    

if __name__ == '__main__':
    # start the interface node
    rospy.init_node('xplane_ros_wrapper', anonymous=True)
    with xpc.XPlaneConnect(timeout=20000) as client:
        ''' instantiate reader and sender objects '''
        

        stateReader = StateReader(client)
        commandSender = CommandSender(client)
        trafficSender = TrafficSender(client)
        resetSim = ResetSim(client)

        
        '''instantiate wrapper object'''
        xplaneRosWrapper = XPlaneRosWrapper(stateReader, commandSender, trafficSender)
        posi = [40.774548,-79.959237, 400, 0,    0,  70,  1]
        # resetSim.reset(posi, 50)
        rospy.spin() #!/usr/bin/env python3


