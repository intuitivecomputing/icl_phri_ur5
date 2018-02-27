#!/usr/bin/env python
from __future__ import division, print_function
import sys
import numpy as np
from time import sleep
import argparse

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

class RobotiqActionClient:
    def __init__(self):
        self._gripper_name = rospy.get_param('~gripper_name', 'icl_phri_gripper/gripper_controller')
        self._ac = actionlib.SimpleActionClient(self._gripper_name, GripperCommandAction)
        self._goal = GripperCommandGoal()

        # Wait 10 Seconds for the gripper action server to start or exit
        if not self._ac.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - %s Gripper Action Server Not Found" %
                         (self._gripper_name.capitalize(),))
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.clear()

    def send_goal(self, position, effort=100.0, timeout=5.0):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._ac.send_goal(self._goal)
        self._ac.wait_for_result(timeout=rospy.Duration(timeout))
        return self._ac.get_result()
    
    def cancel(self):
        self._ac.cancel_goal()

    def clear(self):
        self._goal = GripperCommandGoal()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--position', '-p', metavar='N', type=float, help='Position in meter.0.0-0.14')
    args = parser.parse_args()
    
    rospy.init_node('gripper_action_client')
    gripper_client = RobotiqActionClient()
    print(gripper_client.send_goal(args.position))

if __name__ == '__main__':
    main()
