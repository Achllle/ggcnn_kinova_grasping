#! /usr/bin/env python

import rospy
import tf.transformations as tft

# put it here because some of the imports require a node to be up...
rospy.init_node('ggcnn_open_loop_grasp')

import numpy as np

import kinova_msgs.msg
import kinova_msgs.srv
import std_msgs.msg
import std_srvs.srv
import geometry_msgs.msg

from helpers.gripper_action_client import set_finger_positions
from helpers.position_action_client import position_client, move_to_position
from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform
from helpers.covariance import generate_cartesian_covariance

MOVING = False  # Flag whether the robot is moving under velocity control.
CURR_Z = 0  # Current end-effector z height.


def robot_wrench_callback(msg):
    # Monitor wrench to cancel movement on collision.
    global MOVING
    if MOVING and msg.wrench.force.z < -7.0:
        MOVING = False
        rospy.logerr('Force Detected. Stopping.')


def robot_position_callback(msg):
    # Monitor robot position.
    global CURR_Z
    CURR_Z = msg.pose.position.z


def move_to_pose(pose):
    # Wrapper for move to position.
    p = pose.position
    o = pose.orientation
    move_to_position([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])


def execute_grasp(mode):
    # Execute a grasp.
    global MOVING
    global CURR_Z
    global start_force_srv
    global stop_force_srv

    # Get the positions.
    msg = rospy.wait_for_message('/ggcnn/out/command', std_msgs.msg.Float32MultiArray)
    d = list(msg.data)

    # Calculate the gripper width.
    grip_width = d[4]
    # Convert width in pixels to mm.
    # 0.07 is distance from end effector (CURR_Z) to camera.
    # 0.1 is approx degrees per pixel for the realsense.
    # NOTE: Achille added 20 mm here to the end to account for different gripper and camera rel. pose
    g_width = 2 * ((CURR_Z + 0.07)) * np.tan(0.1 * grip_width / 2.0 / 180.0 * np.pi) * 1000 + 20
    # Convert into motor positions.
    g = min((1 - (min(g_width, 70)/70)) * (6800-4000) + 4000, 5500)
    set_finger_positions([g, g, g])

    rospy.sleep(0.5)

    # Pose of the grasp (position only) in the camera frame.
    gp = geometry_msgs.msg.Pose()
    gp.position.x = d[0]
    gp.position.y = d[1]
    gp.position.z = d[2]
    gp.orientation.w = 1

    # Convert to base frame, add the angle in (ensures planar grasp, camera isn't guaranteed to be perpendicular).
    # gp_base = convert_pose(gp, 'camera_depth_optical_frame', 'm1n6s300_link_base')
    gp_base = convert_pose(gp, 'camera_depth_optical_frame', 'odom')

    q = tft.quaternion_from_euler(np.pi, 0, d[3])
    gp_base.orientation.x = q[0]
    gp_base.orientation.y = q[1]
    gp_base.orientation.z = q[2]
    gp_base.orientation.w = q[3]

    goal_pub = rospy.Publisher('goal_pose_ggcnn', geometry_msgs.msg.PoseStamped, queue_size=5)
    gp_base_stamped = geometry_msgs.msg.PoseStamped()
    gp_base_stamped.header.frame_id = 'odom'
    gp_base_stamped.pose = gp_base

    goal_pub.publish(gp_base_stamped)

    # for visualization: last arg is the duration of the publishment
    publish_pose_as_transform(gp_base, 'm1n6s300_link_base', 'G', 15)

    rospy.loginfo('gp: {}'.format(gp_base))

    # Offset for initial pose.
    initial_offset = 0.20
    if mode == 'force_control':
        gp_base.position.z += initial_offset

        # Disable force control, makes the robot more accurate.
        stop_force_srv.call(kinova_msgs.srv.StopRequest())

    move_to_pose(gp_base)
    rospy.sleep(0.1)

    if mode == 'force_control':
        # Start force control, helps prevent bad collisions.
        start_force_srv.call(kinova_msgs.srv.StartRequest())

        rospy.sleep(0.25)

        # Reset the position
        gp_base.position.z -= initial_offset

        # Flag to check for collisions.
        MOVING = True

        # Generate a nonlinearity for the controller.
        cart_cov = generate_cartesian_covariance(0)

        # Move straight down under velocity control.
        velo_pub = rospy.Publisher('/m1n6s300_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=1)
        while MOVING and CURR_Z - 0.02 > gp_base.position.z:
            dz = gp_base.position.z - CURR_Z - 0.03   # Offset by a few cm for the fingertips.
            MAX_VELO_Z = 0.08
            dz = max(min(dz, MAX_VELO_Z), -1.0*MAX_VELO_Z)

            v = np.array([0, 0, dz])
            vc = list(np.dot(v, cart_cov)) + [0, 0, 0]
            velo_pub.publish(kinova_msgs.msg.PoseVelocity(*vc))
            rospy.sleep(1/100.0)
    

    MOVING = False

    # close the fingers.
    rospy.sleep(0.1)
    set_finger_positions([8000, 8000, 8000])
    rospy.sleep(0.5)

    if mode == 'force_control':
        # Move back up to initial position.
        gp_base.position.z += initial_offset
        gp_base.orientation.x = 1
        gp_base.orientation.y = 0
        gp_base.orientation.z = 0
        gp_base.orientation.w = 0
        move_to_pose(gp_base)

        stop_force_srv.call(kinova_msgs.srv.StopRequest())

    return


if __name__ == '__main__':

    # Robot Monitors.
    wrench_sub = rospy.Subscriber('/m1n6s300_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, robot_wrench_callback, queue_size=1)
    position_sub = rospy.Subscriber('/m1n6s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, robot_position_callback, queue_size=1)

    # https://github.com/dougsm/rosbag_recording_services
    # start_record_srv = rospy.ServiceProxy('/data_recording/start_recording', std_srvs.srv.Trigger)
    # stop_record_srv = rospy.ServiceProxy('/data_recording/stop_recording', std_srvs.srv.Trigger)

    # Enable/disable force control.
    rospy.loginfo('waiting for force control services to come up')
    rospy.wait_for_service('/m1n6s300_driver/in/start_force_control')
    rospy.wait_for_service('/m1n6s300_driver/in/stop_force_control')
    rospy.loginfo('force control services ready')
    start_force_srv = rospy.ServiceProxy('/m1n6s300_driver/in/start_force_control', kinova_msgs.srv.Start)
    stop_force_srv = rospy.ServiceProxy('/m1n6s300_driver/in/stop_force_control', kinova_msgs.srv.Stop)

    # Home position.
    #move_to_position([0, -0.38, 0.25], [0.99, 0, 0, np.sqrt(1-0.99**2)])
    home_pose = [0.223614305258, -0.139523953199, 0.259922802448], \
                [0.899598777294, 0.434111058712, -0.0245193094015, 0.0408461801708]
    # move_to_position(*home_pose)

    try:
        while not rospy.is_shutdown():

            rospy.sleep(0.5)
            set_finger_positions([0, 0, 0])
            rospy.sleep(0.5)

            option = raw_input("Press 'p' for position control, no checking, any other key for force control (recommended)")
            if option == 'p':
                mode = 'position_control'
            else:
                mode = 'force_control'

            # start_record_srv(std_srvs.srv.TriggerRequest())
            rospy.sleep(0.5)
            execute_grasp(mode)
            # move_to_position(*home_pose)
            rospy.sleep(0.5)
            # stop_record_srv(std_srvs.srv.TriggerRequest())

            raw_input('Press Enter to Complete')
    except rospy.ROSInterruptException:
        rospy.loginfo('shutting down')
