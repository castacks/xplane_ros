#!/usr/bin/env python
import rospy

import tf
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

import numpy as np

class NEDToViz:
    def __init__(self):
        self.odomSub = rospy.Subscriber("/xplane/flightmodel/odom", Odometry, self.odom_callback)
        self.odomVizPub = rospy.Publisher("/xplane/flightmodel/odom_viz", Odometry, queue_size=10)
        rospy.Timer(period=rospy.Duration(0.1), callback=self.visualizer_update)
        self.odom_viz = Odometry()
    
    def odom_callback(self, msg):
        self.odom_viz = msg
        pn = (self.odom_viz.pose.pose.position.x) #/10.0
        pe = (self.odom_viz.pose.pose.position.y) #/10.0
        pd = (self.odom_viz.pose.pose.position.z) #/10.0

        self.odom_viz.pose.pose.position.y = -pe
        self.odom_viz.pose.pose.position.z = -pd

        quat = self.odom_viz.pose.pose.orientation
        (r,p,y) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        p = -p
        y = -y
        quat = tf.transformations.quaternion_from_euler(r,p,y)
        self.odom_viz.pose.pose.orientation.x = quat[0]
        self.odom_viz.pose.pose.orientation.y = quat[1]
        self.odom_viz.pose.pose.orientation.z = quat[2]
        self.odom_viz.pose.pose.orientation.w = quat[3]
    
    def visualizer_update(self, event=None):
        self.odomVizPub.publish(self.odom_viz)

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
