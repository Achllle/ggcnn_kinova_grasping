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

# DEBUG: Achille changed MIN_Z from 0.01 to -0.02 as our robot is mounted ~three cm higher than in their setup
MIN_Z = -0.02
CURR_Z = 0.35
CURR_FORCE = 0.0
GOAL_Z = 0.0

VELO_COV = generate_cartesian_covariance(0)

GRIP_WIDTH_MM = 70
CURR_DEPTH = 350  # Depth measured from camera.

SERVO = False

HOME = [0.223614305258, -0.139523953199, 0.259922802448], \
       [0.899598777294, 0.434111058712, -0.0245193094015, 0.0408461801708]
# HOME = [0, -0.38, 0.35], [0.99, 0, 0, np.sqrt(1-0.99**2)]

LATCHED = False

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
    global SERVO
    global CURR_Z, MIN_Z
    global CURR_DEPTH
    global pose_averager
    global GOAL_Z
    global GRIP_WIDTH_MM
    global VELO_COV
    global LATCHED # if we get under 30cm away from object, keep going there no matter what

    CURR_DEPTH = msg.data[5]

    if SERVO:

        d = list(msg.data)

        # PBVS Method.
        # DEBUG: temporarily increase to 30cm so we can see what happens
        if d[2] > 0.28 and not LATCHED:  # Min effective range of the realsense.

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
            if not LATCHED:
                rospy.loginfo('LATCHING')
                LATCHED = True

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

        ########### Added by Achille: hardcode position to check if vel control is working ok
        # gp_base.position.x = 0.281553024819
        # gp_base.position.y = -0.181809716118
        # gp_base.position.z = -0.054720383531
        # gp_base.orientation.x = 0.9628663237748157
        # gp_base.orientation.y = -0.269978596448629
        # gp_base.orientation.z = -1.6531421198955424e-17
        # gp_base.orientation.w = 5.895855807088036e-17

        # Get the Position of the End Effector in the frame fo the Robot base Link
        g_pose = geometry_msgs.msg.Pose()
	# TODO: analyze this, seems very hardcoded
        g_pose.position.z = 0.03  # Offset from the end_effector frame to the actual position of the fingers.
        g_pose.orientation.w = 1
        p_gripper = convert_pose(g_pose, 'm1n6s300_end_effector', 'm1n6s300_link_base')

        publish_pose_as_transform(gp_base, 'm1n6s300_link_base', 'G', 0.0)

        # Calculate Position Error.
        dx = (gp_base.position.x - p_gripper.position.x)
        dy = (gp_base.position.y - p_gripper.position.y)
        dz = (gp_base.position.z - p_gripper.position.z)

        # Orientation velocity control is done in the frame of the gripper,
        #  so figure out the rotation offset in the end effector frame.
        gp_gripper = convert_pose(gp_base, 'm1n6s300_link_base', 'm1n6s300_end_effector')
        # DEBUG: base link frame for sending the gripper to vertical roll and pitch
        # unit_quat = tft.quaternion_from_euler(3.141592653, 0, 0, 'sxyz')
        # unit_pose = geometry_msgs.msg.Pose()
        # unit_pose.orientation.x = unit_quat[0]
        # unit_pose.orientation.y = unit_quat[1]
        # unit_pose.orientation.z = unit_quat[2]
        # unit_pose.orientation.w = unit_quat[3]
        # unit_gripper = convert_pose(unit_pose, 'm1n6s300_link_base', 'm1n6s300_end_effector')
        pgo = gp_gripper.orientation
        q1 = [pgo.x, pgo.y, pgo.z, pgo.w]
        e = tft.euler_from_quaternion(q1)

        # DEBUG: instead of sending it to the ggcnn roll and pitch,
        # send it to vertical position
        # pgo_u = unit_gripper.orientation
        # q_u = [pgo_u.x, pgo_u.y, pgo_u.z, pgo_u.w]
        # e_u = tft.euler_from_quaternion(q_u)

        dr = 1 * e[0]
        # dr = 0.4 * e[0]
        dp = 1 * e[1]
        # dp = 0.4 * e[1]
        # print dr, dp
        dyaw = 1 * e[2]

        vx = max(min(dx * 2.5, MAX_VELO_X), -1.0*MAX_VELO_X)
        vy = max(min(dy * 2.5, MAX_VELO_Y), -1.0*MAX_VELO_Y)
        vz = max(min(dz - 0.04, MAX_VELO_Z), -1.0*MAX_VELO_Z)

        # Apply a nonlinearity to the velocity
        v = np.array([vx, vy, vz])
        vc = np.dot(v, VELO_COV)

        CURRENT_VELOCITY[0] = vc[0]
        CURRENT_VELOCITY[1] = vc[1]
        CURRENT_VELOCITY[2] = vc[2]

        # CURRENT_VELOCITY[3] = 1 * dp
        # CURRENT_VELOCITY[4] = 1 * dr
        CURRENT_VELOCITY[5] = max(min(1 * dyaw, MAX_ROTATION), -1 * MAX_ROTATION)
        CURRENT_VELOCITY[3] = 0
        CURRENT_VELOCITY[4] = 0
        # CURRENT_VELOCITY[5] = 0


def robot_wrench_callback(msg):
    # Monitor force on the end effector, with some smoothing.
    global CURR_FORCE
    CURR_FORCE = 0.5 * msg.wrench.force.z + 0.5 * CURR_FORCE


def finger_position_callback(msg):
    global SERVO
    global CURRENT_FINGER_VELOCITY
    global CURR_DEPTH
    global CURR_Z
    global GRIP_WIDTH_MM

    # Only move the fingers when we're 200mm from the table and servoing.
    if CURR_Z < 0.200 and CURR_DEPTH > 80 and SERVO:
        # 4000 ~= 70mm
        g = min((1 - (min(GRIP_WIDTH_MM, 70)/70)) * (6800-4000) + 4000, 5500)

        # Move fast from fully open.
        gain = 2
        if CURR_Z > 0.12:
            gain = 5

        err = gain * (g - msg.finger1)
        CURRENT_FINGER_VELOCITY = [err, err, 0]

    else:
        CURRENT_FINGER_VELOCITY = [0, 0, 0]


def robot_position_callback(msg):
    global SERVO
    global CURR_Z
    global CURR_DEPTH
    global CURR_FORCE
    global VELO_COV
    global pose_averager
    global start_record_srv
    global stop_record_srv
    global HOME

    CURR_Z = msg.pose.position.z

    # Stop Conditions.
    # DEBUG: Achille changed curr_force < -5.0 to -8.0
    if CURR_Z < MIN_Z or (CURR_Z - 0.01) < GOAL_Z or CURR_FORCE < -8.0:
        if SERVO:
            
            # DEBUG
            rospy.loginfo('STOPPING: CURR_Z < MIN_Z: {}, (CURR_Z - 0.01) < GOAL_Z: {}, or CURR_FORCE < -8.0: {}'.format(CURR_Z < MIN_Z, (CURR_Z - 0.01) < GOAL_Z, CURR_FORCE < -8.0))

            SERVO = False

            # Grip.
            rospy.sleep(0.1)
            set_finger_positions([8000, 8000])
            rospy.sleep(0.5)

            # Move Home.
            rospy.loginfo('moving home...')
            move_to_position(*HOME)
            rospy.sleep(0.25)

            # stop_record_srv(std_srvs.srv.TriggerRequest())

            raw_input('Press Enter to Complete')

            # Generate a control nonlinearity for this run.
            VELO_COV = generate_cartesian_covariance(0.0)

            # Open Fingers
            set_finger_positions([0, 0])
            rospy.sleep(1.0)

            pose_averager.reset()

            raw_input('Press Enter to Start')

            # start_record_srv(std_srvs.srv.TriggerRequest())
            rospy.sleep(0.5)
            SERVO = True


if __name__ == '__main__':
    rospy.init_node('kinova_velocity_control')

    position_sub = rospy.Subscriber('/m1n6s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, robot_position_callback, queue_size=1)
    finger_sub = rospy.Subscriber('/m1n6s300_driver/out/finger_position', kinova_msgs.msg.FingerPosition, finger_position_callback, queue_size=1)
    wrench_sub = rospy.Subscriber('/m1n6s300_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, robot_wrench_callback, queue_size=1)
    command_sub = rospy.Subscriber('/ggcnn/out/command', std_msgs.msg.Float32MultiArray, command_callback, queue_size=1)

    # https://github.com/dougsm/rosbag_recording_services
    # start_record_srv = rospy.ServiceProxy('/data_recording/start_recording', std_srvs.srv.Trigger)
    # stop_record_srv = rospy.ServiceProxy('/data_recording/stop_recording', std_srvs.srv.Trigger)

    start_force_srv = rospy.ServiceProxy('/m1n6s300_driver/in/start_force_control', kinova_msgs.srv.Start)
    start_force_srv.call(kinova_msgs.srv.StartRequest())

    # Publish velocity at 100Hz.
    velo_pub = rospy.Publisher('/m1n6s300_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=1)
    finger_pub = rospy.Publisher('/m1n6s300_driver/in/finger_velocity', kinova_msgs.msg.FingerPosition, queue_size=1)
    r = rospy.Rate(100)

    # rospy.loginfo('moving home...')
    # move_to_position(*HOME)
    rospy.sleep(0.5)
    set_finger_positions([0, 0])
    rospy.sleep(0.5)

    SERVO = True

    while not rospy.is_shutdown():
        if SERVO:
            finger_pub.publish(kinova_msgs.msg.FingerPosition(*CURRENT_FINGER_VELOCITY))
            velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))
        r.sleep()
