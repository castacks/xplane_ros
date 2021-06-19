#!/usr/bin/env python
import rospy 

import rosplane_msgs.msg as rosplane_msgs
import xplane_ros.msg as xplane_msgs
from std_msgs.msg import Float32

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, Ts=0.01, sigma=0.05, limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts
        self.r_ki = 0.1
        self.sigma = sigma
        self.vehicle_state = rosplane_msgs.State()
        self.r_error = 0.0
        self.r_integrator = 0.0

        self.ctrl = xplane_msgs.Controls()
        self.ctrl.aileron = -998
        self.ctrl.rudder = -998
        self.ctrl.throttle = -998
        self.ctrl.elevator =-998

        self.poseSub =  rospy.Subscriber("/fixedwing/xplane/state", rosplane_msgs.State, self.poseCallback)
        self.controlsPub = rospy.Publisher("/xplane/my_control", xplane_msgs.Controls, queue_size=10)
        self.phiPub = rospy.Publisher("/xplane/phi", Float32, queue_size=10)

        rospy.Timer(period=rospy.Duration(0.02), callback=self.control)
    
    def roll_hold(self, phi_c):
        error = phi_c - self.vehicle_state.phi
        self.r_integrator = self.r_integrator + (self.Ts/2)*(error + self.r_error)

        up = self.kp * error
        ui = self.ki * self.r_integrator
        ud = self.kd * self.vehicle_state.p
    
        delta_a = self.saturate(up+ui+ud,1.0,-1.0)
        delta_a_unsat = up+ui+ud
        if self.r_ki >= 0.00001:
            delta_a_unsat = up+ui+ud
            self.r_integrator += (self.Ts/self.r_ki)*(delta_a - delta_a_unsat)

        self.r_error = error
        return delta_a
    
    def saturate(self, u, upper, lower):
        if u>=upper:
            return upper
        if u <= lower:
            return lower
        return u

    def poseCallback(self,msg):
        self.vehicle_state = msg
        # print("In roll_hold poseCallback")
    
    def control(self, event=None):
        self.ctrl.aileron = self.roll_hold(phi_c=0.0)
        # print(self.ctrl.aileron)
        self.controlsPub.publish(self.ctrl)
        self.phiPub.publish(self.vehicle_state.phi)
    
if __name__=="__main__":
    rospy.init_node('roll_hold_node', anonymous=True)

    rollController = PIDController()
    rospy.spin()
