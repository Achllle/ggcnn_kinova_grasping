import rospy
from visualization_msgs.msg import Marker

def create_text_marker(text, position, orientation=[0, 0, 0, 1], frame_id='/world', scale=[0.3, 0.3, 0.3],
                       color=[1.0, 0.0, 0.0, 1.0], duration=[1, 0]):
    """Create and populate a message of type visualization_msgs.msg/Marker

    :param text: the text you want
    :param position: [x, y, z] in the frame 'frame_id'
    :param orientation: [x, y, z, w] in the frame 'frame_id'
    :param frame_id:
    :param scale:
    :param color: [r, g, b, a]. Make sure a is not set to zero otherwise it will be invisible
    :param duration:
    """
    text_marker = Marker()
    text_marker.header.stamp = rospy.get_rostime()
    text_marker.header.frame_id = frame_id # You can also put text on say the end effector!
    text_marker.type = 9  # this specifies that the marker is a text marker
    text_marker.pose.position.x = position[0]
    text_marker.pose.position.y = position[1]
    text_marker.pose.position.z = position[2]
    text_marker.pose.orientation.x = orientation[0]
    text_marker.pose.orientation.y = orientation[1]
    text_marker.pose.orientation.z = orientation[2]
    text_marker.pose.orientation.w = orientation[3]
    text_marker.scale.x = scale[0]
    text_marker.scale.y = scale[1]
    text_marker.scale.z = scale[2]
    text_marker.color.r = color[0]
    text_marker.color.g = color[1]
    text_marker.color.b = color[2]
    text_marker.color.a = color[3] # make sure 'a' is set to >0, otherwise invisible
    text_marker.lifetime.secs = duration[0]
    text_marker.lifetime.nsecs = duration[1]
    text_marker.text = text

    return text_marker