#!/usr/bin/env python3

from __future__ import print_function

import math
import numpy
import rospy
import sys
import tf2_py as tf2
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# the most basic: If you know the names of the two transforms
def tf_distance_between_two_tfs(tfname_1: string, tfname_2:string):
    """
    Will return x,y,z,r,p,y of the relationship between two tf names
    """
    tf_buffer = tf2_ros.Buffer(cache_time=args.cache_time)
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    try:
        cur_time = rospy.Time.now()
        lookup_time = rospy.Time()
        ts = self.tf_buffer.lookup_transform(tfname_1, tfname_2, lookup_time)
    except tf2.LookupException as ex:
        msg = "At time {}, (current time {}) ".format(lookup_time.to_sec(), cur_time.to_sec())
        rospy.logerr(msg + str(ex))
        return None
    except tf2.ExtrapolationException as ex:
        msg = "(current time {}) ".format(cur_time.to_sec())
        rospy.logerr(msg + str(ex))
        return None
    quat = ts.transform.rotation
    (roll, pitch, yaw) = euler_from_quaternion (quat)
    (x, y, z) = ts.transform.translation
    return {roll: roll, pitch: pitch, yaw: yaw, x: x, y: y, z: z}
