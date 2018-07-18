#! /usr/bin/env python



# [INFO] [1530839618.632247]: grasp pose: position:

#   x: 0.351361089294

#   y: -0.0817251843033

#   z: -0.0391555535354

# orientation:

#   x: 0.960813590415

#   y: -0.277195318274

#   z: -1.69733179632e-17

#   w: 5.8832864404e-17

import rospy

import actionlib

import kinova_msgs.msg

import geometry_msgs.msg

import std_msgs.msg

from helpers.position_action_client import position_client, move_to_position, move_to_home


if __name__ == '__main__':

    rospy.init_node('test_kinova')

    # move to HOME pose
    move_to_home()
    HOME = [0.343614305258, -0.109523953199, 0.259922802448], \
       [0.899598777294, 0.434111058712, -0.0245193094015, 0.0408461801708]
    move_to_position(*HOME)


