#!/usr/bin/env python

from __future__ import print_function
import numpy as np
from time import sleep
import sys
import copy

import rospy
from std_msgs.msg import String, Header, Float64
from geometry_msgs.msg import WrenchStamped, Vector3

from utils import *

class MoveGroup:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("manipulator")

        # self.pose_target = geometry_msgs.msg.Pose()
        ## We create this DisplayTrajectory publisher which is used below to publish
        ## trajectories for RVIZ to visualize.
        # self._display_trajectory_publisher = rospy.Publisher(
        #                                     '/move_group/display_planned_path',
        #                                     moveit_msgs.msg.DisplayTrajectory, 
        #                                     queue_size=1)

        ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
        print("============ Waiting for RVIZ...")
        rospy.sleep(1)
        print("============ Starting ")


    def get_pose(self):
        return self._group.get_current_pose().pose

    # def set_pose_target(self, pose):
    #     # pose_target.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(-1.160, 0, 3.009))
    #     self.pose_target.orientation = pose.orientation
    #     self.pose_target.position.x = pose.position.x
    #     self.pose_target.position.y = pose.position.y
    #     self.pose_target.position.z = pose.position.z

    def shift_pose_target(self, axis, value):
        axis_dict = {'x': 0, 'y': 1, 'z': 2, 'r': 3, 'p': 4, 'y': 5}
        print('ee: %s' % self._group.get_end_effector_link())
        self._group.shift_pose_target(axis_dict[axis], value, self._group.get_end_effector_link())
        #self.pose_target = self.get_pose()
    

    def set_pose_target(self, axis, value):
        axis_dict = {'x': 0, 'y': 1, 'z': 2, 'r': 3, 'p': 4, 'y': 5}
        print('ee: %s' % self._group.get_end_effector_link())
        self._group.set_pose_target(axis_dict[axis], value, self._group.get_end_effector_link())
    # def set_pose_target_from_array(self, pose_array):
    #     # x,y,z,qx,qy,qz,w or xyzrpy
    #     # pose_target.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(-1.160, 0, 3.009))
    #     if pose_array.shape[0]==6:
    #         self.pose_target.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(*pose_array[3:]))
    #     else:
    #         self.pose_target.orientation = geometry_msgs.msg.Quaternion(*pose_array[3:])
    #     self.pose_target.position.x = pose_array[0]
    #     self.pose_target.position.y = pose_array[1]
    #     self.pose_target.position.z = pose_array[2]

    # def update_pose_target(self):
    #     self.pose_target = self.get_pose()

    def plan_move(self):
        print("============ Generating plan")
        self.plan = self._group.plan()

    def go(self):
        self._group.go(wait=True)

    def __del__(self):
        moveit_commander.roscpp_shutdown()


class NaiveLeveling:

    def __init__(self):
        rospy.init_node('naive_leveling', anonymous=True)

        
        # PID Publisher and Subscriber
        self._setpoint_pub = rospy.Publisher('setpoint', Float64, queue_size = 10)

        self._state_pub = rospy.Publisher('state', Float64, queue_size = 10)
				
        # r = rospy.Rate(15)
        self._mg = MoveGroup()

        self._current_state = self._mg.getpose.position.z
	self._state_pub.publish(self._current_state)

        self._control_effort_sub = rospy.Subscriber('control_effort', Float64, self._ce_callback)

        self._wrench_sub = rospy.Subscriber('icl_phri_gripper/wrench_filtered', 
                                            WrenchStamped, 
                                            self._wrench_callback, 
                                            queue_size=1)
        
        print(self._mg.get_pose())
        #self.last_torque = None
        #self.diff = 0
        # self._mg.update_pose_target()
        #r.sleep()

    def _ce_callback(self, data):
        self._mg.shift_pose_target('z', data.data)
        self._mg.plan_move()
        self._mg.go()
        self._current_state = self._mg.getpose.position.z
	self._state_pub.publish(self._current_state)

    def _wrench_callback(self, msg):
        self.torque = msg.wrench.torque.y
        self._torque2setpoint(self.torque)
        self._setpoint_pub.publish(self._setpoint)
        #if self.diff > 0.1:
        #    self._mg.shift_pose_target('z', self._effort)
        #    self._mg.plan_move()
        #    self._mg.go()
        #elif self.diff < -0.1:
        #    self._mg.shift_pose_target('z', self._effort)
        #    self._mg.plan_move()
        #    self._mg.go()
        #print(self.diff)  )
        #print(self._mg.pose_target.position)

    def _torque2setpoint(self):
        if diff < 2.0
            self._setpoint = self._current_state
        elif diff > 20.0
            self._setpoint = self._current_state + 0.18
        else
            self._setpoint = self._current_state + (self.torque-2.0)*0.01

        
if __name__ == '__main__':
    NaiveLeveling()
