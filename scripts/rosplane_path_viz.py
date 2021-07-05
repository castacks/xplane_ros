#!/usr/bin/env python
import rospy

import tf
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rosplane_msgs.msg as rosplane_msgs
from visualization_msgs.msg import Marker

import numpy as np 

'''This class takes the waypoints from rosplane code and visualizes markers in rviz. In addition, a straight line joining 2 waypoints in order is also made to get a rough idea about the path.
NOTE : The straight line is not the theoretical Dubins path that the aircraft is trying to follow
'''
class RosplanePathViz:
    def __init__(self):
        '''Subscribe to the commanded waypoints'''
        self.waypointSub = rospy.Subscriber("/fixedwing/waypoint_path", rosplane_msgs.Waypoint, self.waypoint_callback)

        '''Convert waypoint to odometry and Markers for visualization'''
        self.waypointVizPub = rospy.Publisher("/fixedwing/xplane/waypoint_viz", Odometry, queue_size=10)
        self.waypointLinePub = rospy.Publisher("/fixedwing/xplane/waypoint_line_viz", Marker, queue_size=10)

        rospy.Timer(period=rospy.Duration(1.0), callback=self.waypoint_viz_update)

        '''Maintain a list of waypoints'''
        self.waypoints = []
        '''Put origin as a waypoint ; incase the aircraft is following a flight pattern'''
        self.initial_odom = Odometry()
        self.initial_odom.header.stamp = rospy.Time.now()
        self.initial_odom.header.frame_id = "world"
        self.initial_odom.pose.pose.position.x = 0.0
        self.initial_odom.pose.pose.position.y = 0.0
        self.initial_odom.pose.pose.position.z = 0.0

        self.initial_odom.pose.pose.orientation.w = 1
        self.initial_odom.pose.pose.orientation.x = 0
        self.initial_odom.pose.pose.orientation.y = 0
        self.initial_odom.pose.pose.orientation.y = 0
        self.waypoints.append(self.initial_odom)

        '''Line strip to visualize an approximate path'''
        self.line_strip = Marker()
        self.line_strip.header.frame_id = "world"
        self.line_strip.ns = "rudimentary_waypoint_path"
        self.line_strip.id = 1
        self.line_strip.type = Marker.LINE_STRIP
        self.line_strip.scale.x = 10.0
        self.line_strip.color.g = 1.0 # colour of lines will be green
        self.line_strip.color.a = 1.0 # Opaque
        self.line_strip.pose.orientation.w = 1

        '''Add a point at origin'''
        p = Point()
        p.x = p.y = p.z = 0
        self.line_strip.points.append(p)
    

    def waypoint_callback(self, msg):
        '''Clear all the waypoints if the flag is true'''
        if msg.clear_wp_list:
            self.waypoints = []
            self.line_strip.points = []

        waypoint_viz = Odometry()
        waypoint_viz.header.stamp = rospy.Time.now()
        waypoint_viz.header.frame_id = "world"

        '''Convert NED to XYZ
        Similar to what is done in ned_to_viz.py
        '''
        waypoint_viz.pose.pose.position.x = msg.w[0]
        waypoint_viz.pose.pose.position.y = -msg.w[1]
        waypoint_viz.pose.pose.position.z = -msg.w[2]

        r = 0.0
        p = 0.0
        y = -msg.chi_d # Yaw orientation in rviz is reverse of NED yaw

        '''Add waypoint for marker viz'''
        quat = tf.transformations.quaternion_from_euler(r,p,y)
        waypoint_viz.pose.pose.orientation.x = quat[0]
        waypoint_viz.pose.pose.orientation.y = quat[1]
        waypoint_viz.pose.pose.orientation.z = quat[2]
        waypoint_viz.pose.pose.orientation.w = quat[3]
        self.waypoints.append(waypoint_viz)

        '''Add point for line viz'''
        point = Point()
        point.x = waypoint_viz.pose.pose.position.x
        point.y = waypoint_viz.pose.pose.position.y
        point.z = waypoint_viz.pose.pose.position.z
        self.line_strip.points.append(point)
    
    def waypoint_viz_update(self, event=None):
        n = len(self.waypoints)
        if n > 0:
            for i in range(0,n):
                self.waypointVizPub.publish(self.waypoints[i])
        if n > 1:
            self.waypointLinePub.publish(self.line_strip)
        
if __name__=="__main__":
    rospy.init_node("xp_rosplane_path_viz", anonymous=True)
    rosplanePathViz = RosplanePathViz()
    rospy.spin()