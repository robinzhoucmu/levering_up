#!/usr/bin/env python

import rospy 
import numpy
from geometry_msgs.msg import TransformStamped, Transform
import tf

def callback(data):
    
    trans2_mat = tf.transformations.translation_matrix((data.transform.translation.x,data.transform.translation.y,data.transform.translation.z))
    rot2_mat    = tf.transformations.quaternion_matrix((data.transform.rotation.x,data.transform.rotation.y,data.transform.rotation.z,data.transform.rotation.w))
    mat2 = numpy.dot(trans2_mat, rot2_mat)
    mat3 = numpy.dot(work2vision_mat, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)
    m = TransformStamped()
    m.header.frame_id = '/work_obj'
    m.header.seq = data.header.seq
    m.header.stamp.secs = data.header.stamp.secs
    m.header.stamp.nsecs = data.header.stamp.nsecs
    m.child_frame_id = '/vicon_obj'
    m.transform.translation.x = trans3[0]
    m.transform.translation.y = trans3[1]
    m.transform.translation.z = trans3[2]
    m.transform.rotation.x = rot3[0]
    m.transform.rotation.y = rot3[1]
    m.transform.rotation.z = rot3[2]
    m.transform.rotation.w = rot3[3]
    pub.publish(m)
if __name__ == '__main__':
    
    rospy.init_node('obj_pose_checker',anonymous=True)
    while True:
        try:
            #t = tf_listener.getLatestCommonTime('/work_obj','/viconworld')
            trans_stat , rot_stat = tf_listener.lookupTransform('/work_obj','/viconworld',rospy.Time.now())
            trans1_mat = tf.transformations.translation_matrix(trans_stat)
            rot1_mat   = tf.transformations.quaternion_matrix(rot_stat)
            work2vision_mat = numpy.dot(trans1_mat, rot1_mat)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue 
            
    rospy.Subscriber("/vicon/PrePushObj/PrePushObj", TransformStamped, callback)
    rospy.spin()
