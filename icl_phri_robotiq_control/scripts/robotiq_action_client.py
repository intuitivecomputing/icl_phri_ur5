#!/usr/bin/env python
from __future__ import division, print_function
import sys
import numpy as np
from time import sleep
import argparse

import rospy


from robotiq_utils import *


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--position', '-p', metavar='N', type=float, help='Position in meter.0.0-0.14')
    parser.add_argument('--init', '-i', metavar='0/1', type=int, nargs='?', help='initialization', const=1, default=0)
    args = parser.parse_args()
    
    rospy.init_node('gripper_action_client')
    #gripper_name = rospy.get_param('~gripper_name', 'icl_phri_gripper/gripper_controller')
    gripper_name = rospy.get_param('~gripper_name', 'gripper_controller')
    if args.init: 
        RobotiqGripperCtrl()
        gripper_client = RobotiqActionClient(gripper_name)
        print(gripper_client.send_goal(0.14))
        
    if args.position is not None:
        gripper_client = RobotiqActionClient(gripper_name)
        print(gripper_client.send_goal(args.position))

if __name__ == '__main__':
    main()
