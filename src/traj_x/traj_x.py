from time import sleep


MOTION_PRIMITIVES_FILE = '/home/rbaijal/ROS_WS/xplane_ros_ws/src/xplane_ros/utils/traj_index_0SI.txt'
MOTION_PRIMITIVES = open(MOTION_PRIMITIVES_FILE, 'r')

'''#       vel(m/s)        bank_angle(radian)      Climbrate(ft/min)        0     0       0'''

class TrajX:
    def __init__(self):
        motion_prim = MOTION_PRIMITIVES.readlines()
        self.motion_primitives = []
        '''Save all the 252 motion primitives in variable'''
        for mp in motion_prim:
            self.motion_primitives.append(mp.split())
        self.num_primitives = len(self.motion_primitives)
    
    def primitive_to_command(self, index):
        # if the index is invalid just give a command to keep going straight
        if(index >= self.num_primitives):
            return (self.motion_primitives[0][1], self.motion_primitives[0][2], self.motion_primitives[0][3])
        
        ''' Send the Va, phi, vh_c to the controller'''
        command = (self.motion_primitives[index][1], self.motion_primitives[index][2], self.motion_primitives[index][3])
        return command
        

# trajx = TrajX()
# c = trajx.primitive_to_command(1)
# print(c)