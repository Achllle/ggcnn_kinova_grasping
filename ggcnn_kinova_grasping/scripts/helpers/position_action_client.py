# Borrowed and modified from the kinova-ros examples.

import rospy
import actionlib
import kinova_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import sys

from peanut_moveit.msg import ArmAction, ArmGoal, ArmResult, ArmFeedback
import moveit_commander


def move_to_position(position, orientation):
    """Send a cartesian goal to the action server."""
    # global position_client
    # if position_client is None:
    #     init()

    from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
    pose = Pose()
    # pose.header.frame_id = 'm1n6s300_link_base'
    pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    # position_client.send_goal(goal)
    # print(action_address)
    # print('Sent position goal, waiting for result for max 15s...')
    #
    # if position_client.wait_for_result(rospy.Duration(15.0)):
    #     print(" done waiting.")
    #     return position_client.get_result().success
    # else:
    #     position_client.cancel_all_goals()
    #     print('        the cartesian action timed-out')
    #     return None
    moveit_commander.roscpp_initialize(sys.argv)
    success = False
    while not success:
        try:
            group = moveit_commander.MoveGroupCommander('arm')
            success = True
        except RuntimeError as e:
            rospy.loginfo('trying again')
            rospy.sleep(1)
    group.set_pose_reference_frame('m1n6s300_link_base')
    rospy.loginfo(group.get_planning_frame())
    group.set_pose_target(pose)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

action_address = 'execute_trajectory'
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
