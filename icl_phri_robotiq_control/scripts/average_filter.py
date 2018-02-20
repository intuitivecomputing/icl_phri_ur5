#!/usr/bin/env python

import numpy as np
from time import sleep

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3

from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg

from robotiq_ctrl import ForceTorqueSensor

from utils import *

class QueueSetLength:
    "A container with a first-in-first-out (FIFO) queuing policy with a set length."

    def __init__(self, length):
        self._list = np.zeros(6)
        self._length = length

    def push(self, data):
        "Enqueue the 'item' into the queue"
        # print(self._list)
        self._list = np.vstack( ( data, self._list ) )
        if self._list.shape[0] > self._length:
            self.pop()
        
        return self._list

    def pop(self):
        """
          Dequeue the earliest enqueued item still in the queue. This
          operation removes the item from the queue.
        """
        self._list = self._list[:-1]
        return self._list

    def isEmpty(self):
        "Returns true if the queue is empty"
        return self._list.size == 0

    def average(self):
        return np.mean(self._list, axis=0)


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
    