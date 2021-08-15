import rospy 

import rosplane_msgs.msg as rosplane_msgs

from traj_x.traj_x import TrajX as TrajX

class MyFabulousFunnyNode:
    def __init__(self):
        self.trajX = TrajX()
        self.vehicle_state = rosplane_msgs.State()
        self.controller_command = rosplane_msgs.Controller_Commands()

        self.poseSub =  rospy.Subscriber("/fixedwing/xplane/state", rosplane_msgs.State, self.pose_callback)


    def pose_callback(self, msg):
        self.vehicle_state = msg
    
    def motion_primitive_command(self, index):
        command = self.trajX.primitive_to_command(index)
        self.controller_command.Va_c = command[0]
        self.controller_command.phi_c = command[1]
        self.controller_command.vh_c = command[2]
    
