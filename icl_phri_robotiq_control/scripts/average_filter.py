#!/usr/bin/env python

import numpy as np
from time import sleep

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3

from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg

from robotiq_ctrl import ForceTorqueSensor

from robotiq_utils import *


class AverageFilter:
    def __init__(self, window_size=10, topic_name='robotiq_force_torque_wrench'):
        rospy.init_node('average_filter', anonymous=True)
        rospy.get_param("~window_size", 10)
        self._fts_sub = rospy.Subscriber(
            topic_name, WrenchStamped, self._fts_callback)
        self._filtered_pub = rospy.Publisher('out', WrenchStamped, queue_size=1)
        self._queue = QueueSetLength(window_size)
        rospy.spin()

    def _fts_callback(self, wrench_stamped):
        header = wrench_stamped.header
        force = vector3_to_numpy(wrench_stamped.wrench.force)
        torque = vector3_to_numpy(wrench_stamped.wrench.torque)
        data = np.hstack( (force, torque ) )
        
        self._queue.push(data)
        filtered = self._queue.average()
        self._filtered_pub.publish(array_to_wrench_stamped(header, filtered))


if __name__ == '__main__':
    AverageFilter()
    