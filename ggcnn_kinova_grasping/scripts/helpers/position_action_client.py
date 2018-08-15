# Borrowed and modified from the kinova-ros examples.

import rospy
import actionlib
import kinova_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

from peanut_moveit.msg import ArmAction, ArmGoal, ArmResult, ArmFeedback


def move_to_position(position, orientation):
    """Send a cartesian goal to the action server."""
    global position_client
    if position_client is None:
        init()

    goal = ArmGoal()
    # goal.pose.header = std_msgs.msg.Header(frame_id=('m1n6s300_link_base'))
    goal.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    position_client.send_goal(goal)
    print(action_address)
    print('Sent position goal, waiting for result for max 15s...')

    if position_client.wait_for_result(rospy.Duration(15.0)):
        print(" done waiting.")
        print('Result:\n\tState: {}\n\tStatus: {}\n\tSuccess?: {}'.format(
        position_client.get_state(), position_client.get_goal_status_text(), position_client.get_result().success
    ))
        return position_client.get_result().success
    else:
        position_client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None

action_address = 'arm_action_wrapper'
position_client = None

def move_to_home():
    global position_client
    if position_client is None:
        init()    

    goal = ArmGoal()
    goal.move_target = "Home"
    position_client.send_goal(goal)

    position_client.wait_for_result()
    print('Result:\n\tState: {}\n\tStatus: {}\n\tSuccess?: {}'.format(
        position_client.get_state(), position_client.get_goal_status_text(), position_client.get_result().success
    ))


def init():
    global position_client
    position_client = actionlib.SimpleActionClient(action_address, ArmAction)
    position_client.wait_for_server()
    # move_to_home()
