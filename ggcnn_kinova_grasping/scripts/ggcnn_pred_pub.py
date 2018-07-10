#! /usr/bin/env python

import rospy

import numpy as np

import kinova_msgs.srv
import std_msgs.msg
import geometry_msgs.msg

import tf.transformations as tft

from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform
from helpers.covariance import generate_cartesian_covariance

from helpers.gripper_action_client import set_finger_positions
from helpers.position_action_client import move_to_position

MAX_VELO_X = 0.25
MAX_VELO_Y = 0.15
MAX_VELO_Z = 0.085
MAX_ROTATION = 1.5
CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0]
CURRENT_FINGER_VELOCITY = [0, 0, 0]

MIN_Z = 0.01
CURR_Z = 0.35
CURR_FORCE = 0.0
GOAL_Z = 0.0

VELO_COV = generate_cartesian_covariance(0)

GRIP_WIDTH_MM = 70
CURR_DEPTH = 350  # Depth measured from camera.

class Averager():
    def __init__(self, inputs, time_steps):
        self.buffer = np.zeros((time_steps, inputs))
        self.steps = time_steps
        self.curr = 0
        self.been_reset = True

    def update(self, v):
        if self.steps == 1:
            self.buffer = v
            return v
        self.buffer[self.curr, :] = v
        self.curr += 1
        if self.been_reset:
            self.been_reset = False
            while self.curr != 0:
                self.update(v)
        if self.curr >= self.steps:
            self.curr = 0
        return self.buffer.mean(axis=0)

    def evaluate(self):
        if self.steps == 1:
            return self.buffer
        return self.buffer.mean(axis=0)

    def reset(self):
        self.buffer *= 0
        self.curr = 0
        self.been_reset = True


pose_averager = Averager(4, 3)


def command_callback(msg):
    global CURR_Z, MIN_Z
    global CURR_DEPTH
    global pose_averager
    global GOAL_Z
    global GRIP_WIDTH_MM
    global VELO_COV

    CURR_DEPTH = msg.data[5]

    d = list(msg.data)

    # PBVS Method.
    # DEBUG achille changed range to 30 iso 15 based on experimentation
    if d[2] > 0.30:  # Min effective range of the realsense.

        # Convert width in pixels to mm.
        # 0.07 is distance from end effector (CURR_Z) to camera.
        # 0.1 is approx degrees per pixel for the realsense.
        if d[2] > 0.25:
            GRIP_WIDTH_PX = msg.data[4]
            GRIP_WIDTH_MM = 2 * ((CURR_Z + 0.07)) * np.tan(0.1 * GRIP_WIDTH_PX / 2.0 / 180.0 * np.pi) * 1000

        # Construct the Pose in the frame of the camera.
        gp = geometry_msgs.msg.Pose()
        gp.position.x = d[0]
        gp.position.y = d[1]
        gp.position.z = d[2]
        q = tft.quaternion_from_euler(0, 0, -1 * d[3])
        gp.orientation.x = q[0]
        gp.orientation.y = q[1]
        gp.orientation.z = q[2]
        gp.orientation.w = q[3]

        # Calculate Pose of Grasp in Robot Base Link Frame
        # Average over a few predicted poses to help combat noise.
        gp_base = convert_pose(gp, 'camera_depth_optical_frame', 'm1n6s300_link_base')
        gpbo = gp_base.orientation
        e = tft.euler_from_quaternion([gpbo.x, gpbo.y, gpbo.z, gpbo.w])
        # Only really care about rotation about z (e[2]).
        av = pose_averager.update(np.array([gp_base.position.x, gp_base.position.y, gp_base.position.z, e[2]]))

    else:
        gp_base = geometry_msgs.msg.Pose()
        av = pose_averager.evaluate()

    # Average pose in base frame.
    gp_base.position.x = av[0]
    gp_base.position.y = av[1]
    gp_base.position.z = av[2]
    GOAL_Z = av[2]
    ang = av[3] - np.pi/2  # We don't want to align, we want to grip.
    q = tft.quaternion_from_euler(np.pi, 0, ang)
    gp_base.orientation.x = q[0]
    gp_base.orientation.y = q[1]
    gp_base.orientation.z = q[2]
    gp_base.orientation.w = q[3]

    publish_pose_as_transform(gp_base, 'm1n6s300_link_base', 'G', 0.0)


if __name__ == '__main__':
    rospy.init_node('kinova_velocity_control')

    # position_sub = rospy.Subscriber('/m1n6s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, robot_position_callback, queue_size=1)
    command_sub = rospy.Subscriber('/ggcnn/out/command', std_msgs.msg.Float32MultiArray, command_callback, queue_size=1)

    rospy.loginfo('publishing the frame found by ggcnn!')

    rospy.spin()
