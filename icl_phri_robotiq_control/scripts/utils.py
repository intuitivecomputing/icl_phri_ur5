from __future__ import print_function
import numpy as np
import sys
import copy
from geometry_msgs.msg import WrenchStamped, Vector3, Wrench
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def vector3_to_numpy(vect):
    return np.array([vect.x, vect.y, vect.z])

def array_to_wrench(array):
    msg = Wrench()
    msg.force.x = array[0]
    msg.force.y = array[1]
    msg.force.z = array[2]
    msg.torque.x = array[3]
    msg.torque.y = array[4]
    msg.torque.z = array[5]
    return msg

def array_to_wrench_stamped(header, array):
    msg = WrenchStamped()
    msg.header = header
    msg.wrench = array_to_wrench(array)
    return msg


