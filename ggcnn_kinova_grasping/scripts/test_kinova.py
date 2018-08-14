#! /usr/bin/env python

import rospy
import actionlib
import tf2_ros
import tf.transformations as tft
import numpy as np
import pdb

import kinova_msgs.msg
import kinova_msgs.srv
import geometry_msgs.msg
import std_msgs.msg
from helpers.position_action_client import position_client, move_to_position, move_to_home


if __name__ == '__main__':

    rospy.init_node('test_kinova')

    home_world_rpy = np.array([1.518, -1.525, 1.199]).reshape(3, 1)
    home_world_quat = tft.quaternion_from_euler(*home_world_rpy)
    home_world_rotm = tft.quaternion_matrix(home_world_quat)

    # setup
    rospy.wait_for_service('/m1n6s300_driver/in/start_force_control')
    start_force_srv = rospy.ServiceProxy('/m1n6s300_driver/in/start_force_control', kinova_msgs.srv.Start)
    start_force_srv()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    velo_pub = rospy.Publisher('/m1n6s300_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=1)

    rate = rospy.Rate(100)
    to_fr = 'm1n6s300_link_base'
    from_fr = 'm1n6s300_end_effector'
    while not rospy.is_shutdown():
        # convert home_world to current end effector frame
        try:
            trans = tfBuffer.lookup_transform(from_fr, to_fr, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        quat = trans.transform.rotation
        quat_l = np.array([quat.x, quat.y, quat.z, quat.w])
        ee_R_base = tft.quaternion_matrix(quat_l)

        home_ee_rotm = np.matmul(ee_R_base, home_world_rotm)
        home_ee_rpy = tft.euler_from_matrix(home_ee_rotm)

        print home_ee_rpy

        pv = kinova_msgs.msg.PoseVelocity(0, 0, 0, -home_ee_rpy[1], home_ee_rpy[0], home_ee_rpy[2])
        velo_pub.publish(pv)






