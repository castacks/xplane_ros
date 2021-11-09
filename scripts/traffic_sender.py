#!/usr/bin/env python


import xpc.xpc as xpc

import rospy
import xplane_ros.msg as xplane_msgs

import rosflight_msgs.msg as rosflight_msgs

class TrafficSender():
    def __init__(self, client):
        #self.client =  xpc.XPlaneConnect()
        self.client = client
        self.got_traffic = False

        # Listen on the topic for an traffic commands from the user
        self.trafficSub =  rospy.Subscriber("/xplane/adsb_traffic", xplane_msgs.Traffic, self.sendTraffic)
 
    
    def sendTraffic(self, msg):

        num_agents = len(msg.lat)
        print("Sending " + str(num_agents) + " Traffic!!")

        self.got_traffic = True
        for agent in range(num_agents):
            posi = [msg.lat[agent], msg.lon[agent], msg.alt[agent], 0,    0,   msg.yaw[agent],  0]
            print(msg.yaw[agent])
                    # self.client.sendDREF("sim/operation/override/override_TCAS",0)
            self.client.sendPOSI(posi, agent+1) #zero index plus ownship

            # self.client.sendDREF("sim/multiplayer/position/plane1_the", 90.0)

    def check_traffic(self):
        return self.got_traffic
     
if __name__ == '__main__':
    # start the interface node
    rospy.init_node('xplane_ros_traffic_wrapper', anonymous=True)
    with xpc.XPlaneConnect(timeout=20000) as client:
        
        '''instantiate wrapper object'''

        trafficSender = TrafficSender(client)

        rospy.spin() #!/usr/bin/env python3
