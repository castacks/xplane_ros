#!/usr/bin/env python
import rospy

import tf
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

import numpy as np

'''This class takes the odometry data from StateReader and converts it into appropirate XYZ format.
Frome XPlane, the odometry X Y Z correspond to N E D values respectively. In order to make sense of the visualization, X is left as N, Y is made -E and Z is made -D.
During visualization in rviz, X axis (red) corresponds to north, Y axis (green) corresponds to west and Z axis (blue) corresponds to up.
'''
class NEDToViz:
    def __init__(self):
        self.odomSub = rospy.Subscriber("/xplane/flightmodel/odom", Odometry, self.odom_callback)
        self.odomVizPub = rospy.Publisher("/xplane/flightmodel/odom_viz", Odometry, queue_size=10)
        rospy.Timer(period=rospy.Duration(0.1), callback=self.visualizer_update)
        self.odom_viz = Odometry()

        '''Store initial position of aircraft to visualize runway
        TODO : Add some conditions so that instead of the initial position, we have the info of the runway start ; Will be helpful if we spawn mid air
        '''
        self.initial_odom_viz = None
        self.initialOdomVizPub = rospy.Publisher("/xplane/flightmodel/initial_odom_viz", Odometry, queue_size=10)
    
    def odom_callback(self, msg):
        self.odom_viz = msg
        pn = (self.odom_viz.pose.pose.position.x) 
        pe = (self.odom_viz.pose.pose.position.y) 
        pd = (self.odom_viz.pose.pose.position.z) 

        self.odom_viz.pose.pose.position.y = -pe # Y in rviz is west direction
        self.odom_viz.pose.pose.position.z = -pd # Z in rviz is up direction

        quat = self.odom_viz.pose.pose.orientation
        (r,p,y) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        p = -p # pitch orientation reverses
        y = -y # yaw orientation reverses
        quat = tf.transformations.quaternion_from_euler(r,p,y)
        self.odom_viz.pose.pose.orientation.x = quat[0]
        self.odom_viz.pose.pose.orientation.y = quat[1]
        self.odom_viz.pose.pose.orientation.z = quat[2]
        self.odom_viz.pose.pose.orientation.w = quat[3]

        '''Initial odom is taken in order to get a rough idea about the runway'''
        if self.initial_odom_viz is None:
            self.initial_odom_viz = self.odom_viz 
    
    def visualizer_update(self, event=None):
        self.odomVizPub.publish(self.odom_viz)

        if self.initial_odom_viz is not None:
            self.initialOdomVizPub.publish(self.initial_odom_viz)

if __name__ == '__main__':
    rospy.init_node('xplane_ned_to_rviz')
    visualizer = NEDToViz()
    rospy.spin()
    # broadcaster = tf2_ros.StaticTransformBroadcaster()
    # static_transformStamped =  geometry_msgs.msg.TransformStamped()

    # static_transformStamped.header.stamp = rospy.Time.now()
    # static_transformStamped.header.frame_id = "world"
    # static_transformStamped.child_frame_id = "map"

    # static_transformStamped.transform.translation.x = 0.0
    # static_transformStamped.transform.translation.y = 0.0
    # static_transformStamped.transform.translation.z = 0.0

    # quat = tf.transformations.quaternion_from_euler(np.pi, 0.0, 0.0)

    # static_transformStamped.transform.rotation.x = quat[0]
    # static_transformStamped.transform.rotation.y = quat[1]
    # static_transformStamped.transform.rotation.z = quat[2]
    # static_transformStamped.transform.rotation.w = quat[3]

    # broadcaster.sendTransform(static_transformStamped)
    # rospy.spin()
