import numpy as np
from time import sleep

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3

from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg

from robotiq_ctrl import ForceTorqueSensor


class QueueSetLength:
    "A container with a first-in-first-out (FIFO) queuing policy with a set length."

    def __init__(self, length):
        self._list = []
        self._length = length

    def push(self, item):
        "Enqueue the 'item' into the queue"
        self._list.insert(0, item)
        if len(self._length) > self._length:
            self._list.pop()

    def pop(self):
        """
          Dequeue the earliest enqueued item still in the queue. This
          operation removes the item from the queue.
        """
        return self._list.pop()

    def isEmpty(self):
        "Returns true if the queue is empty"
        return len(self._list) == 0

    def average(self):
        return sum(self._list) / len(self._list)


class AverageFilter:
    def __init__(self, window_size):
        self._window = {}
        self._queue = QueueSetLength(window_size)

    def append(self, data):
        self._window.append(data)


if __name__ == '__main__':
    rospy.init_node('average_filter', anonymous=True)
    force_torque_sensor = ForceTorqueSensor('robotiq_force_torque_wrench')