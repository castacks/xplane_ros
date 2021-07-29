import xpc.xpc as xpc

import rospy
import xplane_ros.msg as xplane_msgs
import rosplane_msgs.msg as rosplane_msgs

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

import numpy as np
KNOTS_TO_MS = 0.51444444444
MS_TO_FPM = 60.0/0.305
angle_in_deg = True

'''Class to extract position and controls related information from XPlane '''
class StateReader:
    def __init__(self, client):
        '''instantiate connection to XPC'''
        #self.client =  xpc.XPlaneConnect()
        self.client = client

        self.initPose = Pose()
        self.initPose.position.x = None
        self.initPose.position.y = None
        self.initPose.position.z = None

        # Publish global state consisting of GlobalState msg (Latitutde Longitude instead of openGL coordinates)
        self.globalStatePub = rospy.Publisher("/xplane/flightmodel/global_state", xplane_msgs.GlobalState, queue_size = 10)
        self.odomPub = rospy.Publisher("/xplane/flightmodel/odom", Odometry, queue_size=10)      # Odometry is also being provided in the NED format : XYZ <-> NED

        # self.posePub = rospy.Publisher("/xplane/flightmodel/pose", Pose, queue_size=10)
        # self.velPub = rospy.Publisher("/xplane/flightmodel/velocity", Twist, queue_size=10)
        '''Publisher for data in rosplane format'''
        self.statePub = rospy.Publisher("/fixedwing/xplane/state", rosplane_msgs.State, queue_size=10)

        # self.diff_pub = rospy.Publisher("/xplane/height_diff", Float32, queue_size=10 )
        self.transformPub = rospy.Publisher("/xplane/flightmodel/my_transform", xplane_msgs.TransformedPoint, queue_size=10)
        self.odom = Odometry()



    def sensor_update(self):
        # get global position information from XPlane
        pos = self.client.getPOSI()
        # convert data to ros msgs
        msg = xplane_msgs.Position()
        msg.lat = pos[0]
        msg.lon = pos[1]
        msg.el = pos[2]
        msg.roll = pos[3]
        msg.pitch = pos[4]
        msg.heading = pos[5]
        msg.gear = pos[6]

        self.posePub.publish(msg)
    
    def control_update(self):
        # get control surfaces information from XPlane
        ctrl = self.client.getCTRL()
        # convert data to ros msgs 
        msg = xplane_msgs.Controls()
        msg.elevator = ctrl[0]
        msg.aileron = ctrl[1]
        msg.rudder = ctrl[2]
        msg.throttle = ctrl[3]
        msg.gear = ctrl[4]
        msg.flaps = ctrl[5]
        msg.speed_brakes = ctrl[6]

        self.controlPub.publish(msg)
    
    def sensor_update2(self):
        drefs = []
        ''' Get sim time to generate timestamp and header for ROS msgs '''
        drefs.append("sim/time/total_running_time_sec")

        '''Global latitude (1), longitude(2) and elevation(3) datarefs'''
        drefs.append("sim/flightmodel/position/latitude")
        drefs.append("sim/flightmodel/position/longitude")
        drefs.append("sim/flightmodel/position/elevation")

        '''Position in local coordinates x(4), y(5), z(6)'''
        drefs.append("sim/flightmodel/position/local_x")
        drefs.append("sim/flightmodel/position/local_y")
        drefs.append("sim/flightmodel/position/local_z")

        '''Velocity in local coordinates vx(7), vy(8), vz(9)'''
        drefs.append("sim/flightmodel/position/local_vx")
        drefs.append("sim/flightmodel/position/local_vy")
        drefs.append("sim/flightmodel/position/local_vz")

        ''' attitude information roll(10), pitch(11), yaw(12)'''
        drefs.append("sim/flightmodel/position/phi")
        drefs.append("sim/flightmodel/position/theta")
        drefs.append("sim/flightmodel/position/psi")

        '''Control surface information pitch(13), roll(14), yaw(15), throttle(16), flaps(17), speed brakes(18)'''
        drefs.append("sim/joystick/yoke_pitch_ratio")
        drefs.append("sim/joystick/yoke_roll_ratio")
        drefs.append("sim/joystick/yoke_heading_ratio")
        drefs.append("sim/flightmodel/engine/ENGN_thro")
        drefs.append("sim/flightmodel/controls/flaprat")
        drefs.append("sim/flightmodel/controls/sbrkrat")

        ''' rotation rate pitch(19), roll(20), yaw(21)'''
        drefs.append("sim/flightmodel/position/Q")
        drefs.append("sim/flightmodel/position/P")
        drefs.append("sim/flightmodel/position/R")

        ''' Gear (22) '''
        drefs.append("sim/aircraft/parts/acf_gear_deploy")

        ''' Quaternion (23)'''
        drefs.append("sim/flightmodel/position/q")

        ''' alpha (24), beta(25) '''
        drefs.append("sim/flightmodel/position/alpha")
        drefs.append("sim/flightmodel/position/beta")

        '''Wind speed (26) and x(27), y(28),z (29) components in openGL'''
        drefs.append("sim/weather/wind_speed_kt")
        drefs.append("sim/weather/wind_now_x_msc")
        drefs.append("sim/weather/wind_now_y_msc")
        drefs.append("sim/weather/wind_now_z_msc")

        '''Airspeed (30) and groundspeed (31) '''
        drefs.append("sim/flightmodel/position/indicated_airspeed")
        drefs.append("sim/flightmodel/position/groundspeed")

        ''' Reference latitude(32) and longitude(33) ''' 
        drefs.append("sim/flightmodel/position/lat_ref")
        drefs.append("sim/flightmodel/position/lon_ref")

        ''' verticle velocity(34) '''
        drefs.append("sim/flightmodel/position/vh_ind")


        data = self.client.getDREFs(drefs)
        '''For indices refer above where we append the datarefs'''

        #print(data[8][0], data[34][0])
        #print(data[34][0])

        '''Set initial position so that this vector can be subtracted from subsequent local positions (Centre the frame)'''
        if (not self.initPose.position.x):
            self.initPose.position.x = data[4][0]
            self.initPose.position.y = data[5][0]
            self.initPose.position.z = data[6][0]
            self.opengl_point_to_ned(self.initPose)

        self.global_state = xplane_msgs.GlobalState() # Global coordinate information
        '''Additional 0 index because the data is in the form of a tuple'''
        self.global_state.latitude = data[1][0]
        self.global_state.longitude = data[2][0]
        self.global_state.elevation = data[3][0]
        self.global_state.roll = data[10][0]
        self.global_state.pitch = data[11][0]
        self.global_state.heading = data[12][0]
        self.global_state.gear = data[22][0]

        pose = Pose() # position in local coordinates
        velocity = Twist() # velocity in local coordinates
        odom = Odometry()

        '''pose and orientation in openGL coordinates. However, the angle convention is still NED so no change required there'''
        pose.position.x = data[4][0] 
        pose.position.y = data[5][0]
        pose.position.z = data[6][0]
        pose.orientation.x = data[23][1]
        pose.orientation.y = data[23][2]
        pose.orientation.z = data[23][3]
        pose.orientation.w = data[23][0]

        ''' Quaternion test '''
        # q = quaternion_from_euler(self.global_state.roll * np.pi/180.0, self.global_state.pitch * np.pi/180.0, self.global_state.heading * np.pi/180.0)
        # print(q)
        # print(pose.orientation)
        # print("-----------------------")
        ''' Current data seems good '''

        ''' Convert openGL (East Up South) to NED frame & apply translation''' 
        self.opengl_point_to_ned(pose)
        self.shift_point(pose, self.initPose)
        '''Although linear velocities must be transformed but it seems like XPlane provides Attitude rates according to conventional NED format'''
        velocity.linear.x = data[7][0]    # pn_dot
        velocity.linear.y = data[8][0]    # pe_dot
        velocity.linear.z = data[9][0]    # pd_dot
        velocity.angular.x = data[20][0]  # Roll rate
        velocity.angular.y = data[19][0]  # Pitch rate
        velocity.angular.z =  data[21][0] # Yaw rate
        self.opengl_velocity_to_ned(velocity)

        odom.header.frame_id = '/world'
        '''TODO : In order to be able to plot on rqt with other data, we should instead use Time.now()'''
        odom.header.stamp = rospy.Time(secs=data[0][0])
        odom.pose.pose = pose
        odom.twist.twist = velocity
        self.odom = odom

        ''' rosplane state '''
        state = rosplane_msgs.State()
        state = self.get_rosplane_state(data)
        # state.header.stamp = rospy.Time(secs=data[0][0])
        state.header.stamp = rospy.Time.now()
        # state.header.frame_id = "\x01"
        state.header.frame_id = "world"
        # state.initial_alt = 0.0
        # state.initial_lat = 0.0
        # state.initial_lon = 0.0
        # state.quat_valid = False
        # state.quat = [1.0, 0.0, 0.0, 0.0]
        # state.chi_deg = 0.0
        # state.psi_deg = 0.0

        '''Print statements to check if local_vy can be used as vertical velocity indicator
        vh_ms = self.client.getDREF("sim/flightmodel/position/vh_ind")[0]
        vh_fpm = self.client.getDREF("sim/flightmodel/position/vh_ind_fpm")[0]
        print("sensor : %f, %f, %f" % (vh_ms, -velocity.linear.z, vh_fpm*0.3048/60))'''


        self.globalStatePub.publish(self.global_state)
        self.odomPub.publish(odom)
        # self.posePub.publish(pose)
        # self.velPub.publish(velocity)
        self.statePub.publish(state)

        # self.diff_pub.publish(data[5][0] - data[3][0])
        '''TODO : local_vx, vy, vz don't seem to give a magnitude equal to airspeed. It could be Vg instead ; investigate this'''

    
    def get_rosplane_state(self, data):
        state = rosplane_msgs.State()
        state.position[0] = -data[6][0] - self.initPose.position.x
        state.position[1] = data[4][0] - self.initPose.position.y
        state.position[2] = -data[5][0] - self.initPose.position.z

        state.Va =  data[30][0] * KNOTS_TO_MS # dataref gives airspeed in knots; need to convert it to m/s

        '''Sending angle values in degrees or in rad'''
        if angle_in_deg:
            state.alpha = data[24][0]
            state.beta = data[25][0]

            state.phi = data[10][0] 
            state.theta = data[11][0] 
            state.psi =  data[12][0] 
        else:
            state.alpha = data[24][0] * (np.pi / 180.0)
            state.beta = data[25][0] * (np.pi / 180.0)

            state.phi = data[10][0] * (np.pi/180)
            state.theta = data[11][0] * (np.pi/180)
            state.psi =  data[12][0] * (np.pi/180)

        state.p = data[20][0] * (np.pi/180) # roll rate in rad/s
        state.q = data[19][0] * (np.pi/180) # pitch rate in rad/s
        state.r = data[21][0] * (np.pi/180) # yaw rate in rad/s

        state.Vg = data[31][0] # dataref gives groundspeed in m/s
        wind_speed = data[26][0]
        '''wn = w * -z_component
        we = w * x_component '''
        state.wn = wind_speed * (-data[29][0])
        state.we = wind_speed * (data[27][0])
        # state.wn = 0
        # state.we = 0
        state.vh = data[8][0] * MS_TO_FPM 

        '''Print statements to see if speed is in m/s or knots'''
        # vx = self.odom.twist.twist.linear.x 
        # vy = self.odom.twist.twist.linear.y 
        # vz = self.odom.twist.twist.linear.z
        # print("Airspeed Xplane : %f" % (state.Va))
        # print("Airpspeed in m/s %f" % (state.Va * 0.51444444444 ))
        # print("Self Airspeed : %f" % (np.sqrt(vx*vx + vy*vy + vz*vz)))
        # print("Ground velocity : %f" % (state.Vg))
        # print("Ground velocity in m/s : %f" % (state.Vg * 0.51444444444 ))
        # print("self Groundspeed : %f" % (np.sqrt(vx*vx + vy*vy)))
        # print("-------------------------------------")
        '''Observations :
        grounndspeed dataref infact gives groundspeed in m/s.
        But airspeed is in knots.
        sqrt(vx*vx  + vy*vy + vz*vz) = groundspeed (Shoudl've been equal to airspeed)
        airspeed seems slightly off from sqrt(vx*vx + vy*vy + vz*vz) probably because of wind
        '''

        state.chi = state.psi + state.beta # TODO : calculate course angle ; currently assume wind velocity is 0
        if angle_in_deg:
            if state.chi > 180.0:
                state.chi = state.chi - 2*180.0
            if state.chi < -180.0:
                state.chi = state.chi + 2*180.0
            
            '''Wrap the course angle between -PI and PI'''
            if state.psi > 180.0:
                state.psi -= 2*180.0
            if state.psi < -180.0:
                state.psi += 2*180.0
        else:
            if state.chi > np.pi:
                state.chi = state.chi - 2*np.pi
            if state.chi < -np.pi:
                state.chi = state.chi + 2*np.pi
            
            '''Wrap the course angle between -PI and PI'''
            if state.psi > np.pi:
                state.psi -= 2*np.pi
            if state.psi < -np.pi:
                state.psi += 2*np.pi

        
        return state

    def opengl_point_to_ned(self, pose):
        ''' [pn,pe,pd]^T =  [0, 0, -1] [x,y,z]^T  
                            [1, 0, 0 ]
                            [0, -1, 0]   '''
        pn = -pose.position.z
        pe = pose.position.x
        pd = -pose.position.y
        pose.position.x = pn
        pose.position.y = pe
        pose.position.z = pd
    
    def opengl_velocity_to_ned(self, vel):
        ''' [pn,pe,pd]^T =  [0, 0, -1] [x,y,z]^T  
                            [1, 0, 0 ]
                            [0, -1, 0]   '''
        pn_dot = -vel.linear.z
        pe_dot = vel.linear.x
        pd_dot = -vel.linear.y
        vel.linear.x = pn_dot
        vel.linear.y = pe_dot
        vel.linear.z = pd_dot
    
    def shift_point(self, pose, init):
        pose.position.x = (pose.position.x - init.position.x)
        pose.position.y = (pose.position.y - init.position.y)
        pose.position.z = (pose.position.z - init.position.z)
