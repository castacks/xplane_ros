import rospy 

import rosplane_msgs.msg as rosplane_msgs

from traj_x.traj_x import TrajX as TrajX

class MyFabulousFunnyNode:
    def __init__(self):
        '''Helper functions for easy conversions of trajectory library to commands'''
        self.trajX = TrajX()

        '''Vehicle state in rosplane format'''
        self.vehicle_state = rosplane_msgs.State()

        '''Controller commands in rosplane format'''
        self.controller_command = rosplane_msgs.Controller_Commands()

        '''Subscriber for vehicle state'''
        self.pose_sub =  rospy.Subscriber("/fixedwing/xplane/state", rosplane_msgs.State, self.pose_callback)

        '''Publisher for controller commands'''
        self.controller_pub = rospy.Publisher("/fixedwing/ai_controller_commands", rosplane_msgs.Controller_Commands, queue_size=10)


    def pose_callback(self, msg):
        self.vehicle_state = msg
    
    def motion_primitive_command(self, index):
        '''conversion of trajectory library motion primitive to Controller_Command format'''
        command = self.trajX.primitive_to_command(index)
        self.controller_command.Va_c = command[0]
        self.controller_command.phi_c = command[1]
        self.controller_command.vh_c = command[2]
    
