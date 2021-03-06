#!/usr/bin/env python

# XPC
import xpc.xpc as xpc 
from state_reader import StateReader
from reset_sim import ResetSim
#ROS
import rospy 


class XPlaneRosWrapper:
    def __init__(self, stateReader):
        self.stateReader = stateReader
        # self.commandSender = commandSender
        '''single function for all the information updates to avoid the issue of synchronization'''
        # rospy.Timer(period=rospy.Duration(0.1), callback=self.sensor_update)

        self.sensor_update()

    def sensor_update(self):
        '''Extract all the useful information sequentially here'''
        rate = rospy.Rate(10)

        if self.stateReader is not None:
            # self.stateReader.sensor_update()
            # self.stateReader.control_update()
            
            while not rospy.is_shutdown():
                # print("XPlaneROS wrapper is Running")

                self.stateReader.sensor_update2()
                rate.sleep()


if __name__ == '__main__':
    # start the interface node
    rospy.init_node('xplane_ros_wrapper', anonymous=True)
    with xpc.XPlaneConnect(timeout=20000) as client:
        ''' instantiate reader and sender objects '''
        
        resetSim = ResetSim(client)

        stateReader = StateReader(client)
        
        '''instantiate wrapper object'''
        xplaneRosWrapper = XPlaneRosWrapper(stateReader)
        rospy.spin() #!/usr/bin/env python3


