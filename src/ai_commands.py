#!/usr/bin/env python

import rospy 
import rosplane_msgs.msg as rosplane_msgs
import xplane_ros.msg as xplane_msgs
<<<<<<< HEAD

from matplotlib import pyplot as plt
from traj_x.traj_x import TrajX as TrajX
from xplane_ros_utils.utils import read_traffic_file, to_local_runway_frame

import random

=======
import xplane_ros.srv as xplane_srv
from matplotlib import pyplot as plt
from traj_x.traj_x import TrajX as TrajX
from utils import *
import time
>>>>>>> traffic-jay
class AICNode:
    def __init__(self,plot=True, traffic_path=None):
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

<<<<<<< HEAD
        '''Publisher for traffic'''
        self.traffic_pub = rospy.Publisher("/fixedwing/adsb_traffic", xplane_msgs.Traffic, queue_size=10)
        self.traffic = xplane_msgs.Traffic()

        rospy.Timer(period=rospy.Duration(10), callback=self.compute_motion_primitive)
        rospy.Timer(period=rospy.Duration(0.1), callback=self.send_custom_command)
=======
        self.reset_sim = rospy.ServiceProxy('reset_sim',xplane_srv.ResetSim)

        
        '''Publisher for traffic'''
        self.traffic_pub = rospy.Publisher("/xplane/adsb_traffic", xplane_msgs.Traffic, queue_size=1)
        self.traffic = xplane_msgs.Traffic()
>>>>>>> traffic-jay
        
        self.plot = plot

        self.plot_counter = 0
<<<<<<< HEAD

        self.primitive_index = 0 
        
        self.got_traffic = False
=======
        
        self.got_traffic = False
        # self.got_curr_traffic = False
>>>>>>> traffic-jay
        if traffic_path is not None:
            self.read_traffic(traffic_path)
            self.got_traffic = True

<<<<<<< HEAD

    def read_traffic(self,path):

=======
    def reset_sim_client(self):

        rospy.wait_for_service('reset_sim')
        self.start_time = rospy.get_time()

        try:
            # posi = [40.774548,-79.959237, 400, 0,    0,  70,  1]
            resp = self.reset_sim(40.774548,-79.959237, 400,70,50)
        except:
            print("Reset Service Failed!")

    def read_traffic(self,path):
        print("Reading Traffic....")
>>>>>>> traffic-jay
        self.traffic_data, self.ownship = read_traffic_file(path)


    def pose_callback(self, msg):
        self.vehicle_state = msg
        if self.plot_counter%10 ==0 and self.plot:
<<<<<<< HEAD
            x,y = to_local_runway_frame(msg.latitude,msg.longitude)
            plt.plot(x,y,'*',color='r')
=======
            plt.clf()
            plt.rcParams["figure.figsize"] = (5,5)

            # for h in hh:
                # h.remove()
            x,y = to_local_runway_frame(msg.latitude,msg.longitude)
            plt.plot(x,y,'*',color='r')
            try:
                for agent in self.curr_traffic:
                    print("Agent : ", agent)
                    plt.plot(self.curr_traffic[agent]['x'],self.curr_traffic[agent]['y'],"*",color='k')
            except:
                print("No Traffic")
>>>>>>> traffic-jay
            plt.scatter(0,0,color='k')
            plt.scatter(1.45,0,color='k') 
            plt.plot([0,1.450],[0,0],'--',color='k')
            plt.axis("equal")
<<<<<<< HEAD
=======
            plt.grid(True)
            plt.xlim([-5,5])
>>>>>>> traffic-jay
            plt.draw()
            plt.pause(0.000001)
            
        self.plot_counter +=1 

    def motion_primitive_command(self, index):
        '''conversion of trajectory library motion primitive to Controller_Command format'''
        command = self.trajX.primitive_to_command(index)
        self.controller_command.Va_c = float(command[0])
        self.controller_command.phi_c = float(command[1])
        self.controller_command.vh_c = float(command[2])
<<<<<<< HEAD
        print("Va",float(command[0]),"Roll",float(command[1]) ,"VV",float(command[2]))
    
    # def send_custom_commands(self):
    #     '''send custom commands for debug'''
    #     print("Sending Custom Commands")
    #     rate = rospy.Rate(1)
    #     self.start_time = rospy.get_time()
    #     while not rospy.is_shutdown():
    #         if self.got_traffic:
    #             self.send_traffic()
    #         if rospy.get_time() - self.start_time < 20:
    #             self.motion_primitive_command(0)
    #         else:
    #             self.motion_primitive_command(119)
                

    #         self.controller_pub.publish(self.controller_command)
    #         rate.sleep()

    def compute_motion_primitive(self, step):
        '''Run the DL network or other classical algorithm here'''
        self.primitive_index = random.randint(0,255)

    def send_custom_command(self, step):
        print("Sending motion primitive : ", self.primitive_index)
        self.motion_primitive_command(self.primitive_index)
        self.controller_pub.publish(self.controller_command)

    def send_traffic(self):
        '''send traffic'''
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            curr_traffic = self.traffic_data[int(rospy.get_time() - self.start_time)]
            # print(curr_traffic)
            # self.traffic.lat = 
            # self.traffic_pub.publish()
=======
        # print("Va",float(command[0]),"Roll",float(command[1]) ,"VV",float(command[2]))
    
    def send_custom_commands(self):
        '''send custom commands for debug'''
        print("Sending Custom Commands")
        rate = rospy.Rate(5)
        count = 0
        while not rospy.is_shutdown():
            if self.got_traffic and not count%1:
                self.send_traffic()
            if rospy.get_time() - self.start_time < 30:
                self.motion_primitive_command(0)
            else:
                self.motion_primitive_command(0)
                
            count = count + 1
            self.controller_pub.publish(self.controller_command)
            rate.sleep()

    def send_traffic(self):
        '''publish traffic to ros'''
        self.curr_traffic = self.traffic_data[500+int(rospy.get_time() - self.start_time)]
        self.prev_traffic = self.traffic_data[500+int(rospy.get_time() - self.start_time)-1]

        all_lat = []
        all_lon = []
        all_alt = []
        all_yaw = []
        for agent in self.curr_traffic:
            sol = to_global_frame(self.curr_traffic[agent]['x'],self.curr_traffic[agent]['y'])
            all_lat.append(sol['lat2'])
            all_lon.append(sol['lon2'])
            all_alt.append(max(379.476,self.curr_traffic[agent]['z']*1000))
            yaw = np.arctan2(self.curr_traffic[agent]['y']-self.prev_traffic[agent]['y'],self.curr_traffic[agent]['x']-self.prev_traffic[agent]['x'])
            all_yaw.append(np.rad2deg(yaw)+72)

        self.traffic.lat = all_lat
        self.traffic.lon = all_lon
        self.traffic.alt = all_alt
        self.traffic.yaw = all_yaw
        # print(self.traffic)
        self.traffic_pub.publish(self.traffic)
>>>>>>> traffic-jay
            

    def send_ground_truth_commands(self):
        pass

        


if __name__ == '__main__':
    rospy.init_node('ai_commands', anonymous=True)
    print("AI Commands Initiated")
<<<<<<< HEAD
    path ='/home/rbaijal/ROS_WS/xplane_ros_ws/src/xplane_ros/utils/304.txt'

    aicnode = AICNode(traffic_path = path)
    # aicnode.send_custom_command()
   
    # plt.ion()
    # plt.grid(True)
    # plt.show()

    rospy.spin()  #!/usr/bin/env python3
=======
    # path ='/home/jay/xplane_ros_ws/src/xplane_ros/utils/304.txt'
    path ='/home/rbaijal/ROS_WS/xplane_ros_ws/src/xplane_ros/utils/304.txt'
    aicnode = AICNode(traffic_path = path)
    # time.sleep(5)
    aicnode.reset_sim_client()
    aicnode.send_custom_commands()
   
    plt.ion()
    plt.grid(True)
    plt.show()

    # rospy.spin()  #!/usr/bin/env python3
>>>>>>> traffic-jay
