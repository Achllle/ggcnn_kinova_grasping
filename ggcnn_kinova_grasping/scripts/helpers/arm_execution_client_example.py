#!/usr/bin/env python
"""
All Rights Reserved, Peanut Robotics 2018
"""

import rospy
import actionlib
import rospkg
import signal
import os

from geometry_msgs.msg import Pose
from peanut_moveit.msg import ArmAction, ArmGoal, ArmResult, ArmFeedback

def feedback_cb(feedback):
    """Print feedback from action server
    Args:
        feedback: feedback object as defined in action/Voice.cfg
    """
    print("Feedback:\n\tCurrent Pose: ".format(
        feedback.current.pose
    ))

    print("starting peanut_arm_execution")

def fake_pose():
    '''
        A function to initialize a fake pose for the robot to plan to.
    '''
    pose = Pose()
    pose.position.x = 0.223160952089
    pose.position.y = -0.264720846045
    pose.position.z = 0.0354122434776

    pose.orientation.x = -0.400142782748
    pose.orientation.y = 0.916433444637
    pose.orientation.z = -0.00570341780644
    pose.orientation.w = 0.00172220502379

    return pose


if __name__ == '__main__':
    '''
    An example of interfacing with the arm execution wrapper action server.
    '''
    rospy.init_node('example_peanut_arm_planner_client')

    rospkg.RosPack()
    
    # setup safe termination
    def handler(signum, frame):
        rospy.loginfo('caught CTRL+C, exiting...')           
        exit(0)
    signal.signal(signal.SIGINT, handler)

    # Start the action client
    rospy.loginfo("starting action client")
    client = actionlib.SimpleActionClient("arm_action_wrapper", ArmAction)
    client.wait_for_server()

    # build the request to the action service
    goal = ArmGoal()
    goal.pose = fake_pose()
    goal.move_target = "Vertical"
    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()
    print('Result:\n\tState: {}\n\tStatus: {}\n\tSuccess?: {}'.format(
        client.get_state(), client.get_goal_status_text(), client.get_result().success
    ))