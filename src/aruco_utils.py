#!/usr/bin/env python3

import rospy
import sys
import tf2_py as tf2
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# the most basic: If you know the names of the two transforms
def tf_distance_between_two_tfs(tfname_1: str, tfname_2: str):
    """
    Will return x,y,z,r,p,y of the relationship between two tf names
    """
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(1.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(6.0)
    try:
        cur_time = rospy.Time.now()
        ts = tf_buffer.lookup_transform(tfname_1, tfname_2, rospy.Time(0))
    except tf2.LookupException as ex:
        rospy.logerr(str(ex))
        return None
    except tf2.ExtrapolationException as ex:
        rospy.logerr(str(ex))
        return None
    quat = ts.transform.rotation
    (roll, pitch, yaw) = euler_from_quaternion ([quat.x, quat.y, quat.z, quat.w])
    trans = ts.transform.translation
    return {"roll": roll, "pitch": pitch, "yaw": yaw, "x": trans.x, "y": trans.y, "z": trans.z}

def tf_list_all():
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(5.0)
    return (tf_buffer.all_frames_as_string())

# Here's the main used for testing
if __name__ == '__main__':
    rospy.init_node("aruco_utils")
    #print(tf_distance_between_two_tfs("raspicam","fiducial_2"))
    print(tf_list_all())

