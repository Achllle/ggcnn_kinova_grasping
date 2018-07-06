# Borrowed and modified from the kinova-ros examples.

import rospy
import actionlib
import kinova_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import moveit_commander
import sys

from geometry_msgs.msg import Pose


def move_to_position(position, orientation):
    """Send a cartesian goal to the action server."""
    global position_client
    if position_client is None:
        init()

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

    # global position_client
    # if position_client is None:
    #     init()

    # goal = kinova_msgs.msg.ArmPoseGoal()
    # goal.pose.header = std_msgs.msg.Header(frame_id=('m1n6s300_link_base'))
    # goal.pose.pose.position = geometry_msgs.msg.Point(
    #     x=position[0], y=position[1], z=position[2])
    # goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
    #     x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    # pose = Pose()
    # pose.position.x = position[0]
    # pose.position.y = position[1]
    # pose.position.z = position[2]
    #
    # pose.orientation.x = orientation[0]
    # pose.orientation.y = orientation[1]
    # pose.orientation.z = orientation[2]
    # pose.orientation.w = orientation[3]
    #
    # position_client.set_pose_target(pose)
    # found_plan = position_client.plan()
    #
    # raw_input('press any key to continue: ')
    #
    # if found_plan.joint_trajectory.joint_names != []:
    #     position_client.execute(found_plan, wait=True)
    #     success = True
    # else:
    #     success = False

action_address = '/m1n6s300_driver/pose_action/tool_pose'
position_client = None
# position_client = None

def init():
    global position_client
    position_client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    position_client.wait_for_server()
    # moveit_commander.roscpp_initialize(sys.argv)
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # global position_client
    # position_client = moveit_commander.MoveGroupCommander("arm")

