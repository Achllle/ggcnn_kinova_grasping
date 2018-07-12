#! /usr/bin/env python
import sys
import rospy
import numpy as np

import actionlib
import moveit_commander

import moveit_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from peanut_moveit.msg import ArmAction, ArmGoal, ArmResult, ArmFeedback

class MoveitInterfacer(object):
    """Interface between higher lvl code and moveit
    requires moveit to be launched and controllers to be up
    """

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")

        # for publishing trajectories to Rviz for visualization
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.action_wrapper_server = actionlib.SimpleActionServer('arm_action_wrapper',
                                                                  ArmAction,
                                                                  auto_start=False)

        self.action_wrapper_server.register_preempt_callback(self.aws_preempt_cb)
        self.action_wrapper_server.register_goal_callback(self.aws_goal_cb)
        self.action_wrapper_server.start()

    def aws_goal_cb(self):
        """
        send the pose to moveit when a new goal comes in
        :param goal: ArmGoal
        """
        goal = self.action_wrapper_server.accept_new_goal()

        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()

        if goal.move_target is "":
            self.group.set_pose_target(goal.pose)

        if goal.move_target is not "":
            self.group.set_named_target(goal.move_target)

        plan = self.group.plan()

        found_plan = self.group.plan()

        if found_plan.joint_trajectory.joint_names != []:
            self.group.execute(found_plan, wait=True)
            success = True
        else:
            success = False

        self.group.clear_pose_targets()

        result = ArmResult()

        result.success = success

        self.action_wrapper_server.set_succeeded(result,"Successfully moved to pose")

        return success

    def aws_preempt_cb(self):
        """Pre-emption signal to stop execution.
        Executed when a new goal is sent
        """
        self.group.stop()
        self.action_wrapper_server.set_preempted()
        rospy.loginfo('preempting previously running arm action.')


if __name__ == "__main__":
    rospy.init_node('peanut_moveit_interface')
    mi = MoveitInterfacer()
    rospy.spin()