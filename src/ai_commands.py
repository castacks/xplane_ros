#!/usr/bin/env python

import rospy 
import rosplane_msgs.msg as rosplane_msgs
import xplane_ros.msg as xplane_msgs

from matplotlib import pyplot as plt
from traj_x.traj_x import TrajX as TrajX
from utils import read_traffic_file, to_local_runway_frame

class AICNode:
    def __init__(self,plot=True):
        '''Helper functions for easy conversions of trajectory library to commands'''
        self.trajX = TrajX()

        '''Vehicle state in rosplane format'''
        self.vehicle_state = rosplane_msgs.State()

        '''Controller commands in rosplane format'''
        self.controller_command = rosplane_msgs.Controller_Commands()

        '''Subscriber for vehicle state'''
        self.pose_sub =  rospy.Subscriber("/xplane/flightmodel/global_state", xplane_msgs.GlobalState, self.pose_callback)

        '''Publisher for controller commands'''
        self.controller_pub = rospy.Publisher("/fixedwing/ai_controller_commands", rosplane_msgs.Controller_Commands, queue_size=10)

        self.read_traffic()

        self.plot = plot

    def read_traffic(self):

        path ='/home/jay/xplane_ros_ws/src/xplane_ros/utils/304.txt'

        read_traffic_file(path)


    def pose_callback(self, msg):
        self.vehicle_state = msg
        global counter
        if counter%10 ==0 and self.plot:
            x,y = to_local_runway_frame(msg.latitude,msg.longitude)
            plt.plot(x,y,'*',color='r')
            plt.scatter(0,0,color='k')
            plt.scatter(1.45,0,color='k') 
            plt.plot([0,1.450],[0,0],'--',color='k')
            plt.axis("equal")
            plt.draw()
            plt.pause(0.000001)
            
        counter +=1

    def motion_primitive_command(self, index):
        '''conversion of trajectory library motion primitive to Controller_Command format'''
        command = self.trajX.primitive_to_command(index)
        self.controller_command.Va_c = float(command[0])
        self.controller_command.phi_c = float(command[1])
        self.controller_command.vh_c = float(command[2])
        print("Va",float(command[0]),"Roll",float(command[1]) ,"VV",float(command[2]))
    
    def send_custom_commands(self):
        '''send custom commands for debug'''
        print("Sending Custom Commands")
        rate = rospy.Rate(1)
        now = rospy.get_time()
        while not rospy.is_shutdown():
            if rospy.get_time() - now < 20:
                self.motion_primitive_command(0)
            else:
                self.motion_primitive_command(119)
                

            self.controller_pub.publish(self.controller_command)
            rate.sleep()

    def send_ground_truth_commands(self):
        pass

        


if __name__ == '__main__':
    counter = 0
    rospy.init_node('ai_commands', anonymous=True)
    print("AI Commands Initiated")
    aicnode = AICNode()
    aicnode.send_custom_commands()
   
    plt.ion()
    plt.grid(True)
    plt.show()

    # rospy.spin()  #!/usr/bin/env python3
