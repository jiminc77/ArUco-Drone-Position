#!/usr/bin/env python3

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
import numpy as np

class ArucoMavrosBridge:
    def __init__(self):
        rospy.init_node('aruco_mavros_bridge')
        
        self.aruco_sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.aruco_cb)
        self.mavros_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        
    def aruco_cb(self, msg):
        t_c_m = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q_c_m = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
        mat_c_m = tft.quaternion_matrix(q_c_m)
        mat_c_m[0:3, 3] = t_c_m
        
        mat_m_c = np.linalg.inv(mat_c_m)
        
        t_m_c = mat_m_c[0:3, 3]
        q_m_c = tft.quaternion_from_matrix(mat_m_c)
        
        out_msg = PoseStamped()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = "map"
        
        out_msg.pose.position.x = t_m_c[0]
        out_msg.pose.position.y = t_m_c[1]
        out_msg.pose.position.z = t_m_c[2]
        
        out_msg.pose.orientation.x = q_m_c[0]
        out_msg.pose.orientation.y = q_m_c[1]
        out_msg.pose.orientation.z = q_m_c[2]
        out_msg.pose.orientation.w = q_m_c[3]
        
        self.mavros_pub.publish(out_msg)

if __name__ == '__main__':
    try:
        node = ArucoMavrosBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
