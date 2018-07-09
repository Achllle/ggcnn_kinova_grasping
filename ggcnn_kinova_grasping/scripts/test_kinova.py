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

# from helpers.position_action_client import position_client, move_to_position



def move_to_position(position, orientation):

    """Send a cartesian goal to the action server."""

    action_address = '/m1n6s300_driver/pose_action/tool_pose'

    position_client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)

    position_client.wait_for_server()



    goal = kinova_msgs.msg.ArmPoseGoal()

    goal.pose.header = std_msgs.msg.Header(frame_id=('m1n6s300_link_base'))

    goal.pose.pose.position = geometry_msgs.msg.Point(

        x=position[0], y=position[1], z=position[2])

    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(

        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])



    position_client.send_goal(goal)

    print('Sent position goal: \n {}'.format(goal))



    if position_client.wait_for_result(rospy.Duration(10.0)):

        return position_client.get_result()

    else:

        position_client.cancel_all_goals()

        print('Execution failed: timed out waiting for ArmPoseGoal')

        return None



if __name__ == '__main__':

    rospy.init_node('test_kinova')

    # move to HOME pose
    move_to_position([0.223614305258, -0.139523953199, 0.259922802448],
                     [0.899598777294, 0.434111058712, -0.0245193094015, 0.0408461801708])

    # move to grasp pose:

    # move_to_position([0.351361089294, -0.0817251843033, -0.0391555535354],

    #                  [0.960813590415, -0.277195318274, -1.69733179632e-17, 5.8832864404e-17])

    # or, move to known working pose:

    #move_to_position([0.261, 0.0383, 0.222],
    #                 [0.97, -0.24, -0.0018, 0.0182])

    # move_to_position([0.261, 0.0383, 0.222],

    #                  [-0.52, 0.85, -0.012, -0.016])

    #move_to_position([0.313, 0.05, 0.014],
    #                [0.24, 0.97, -0.026, -0.033])
