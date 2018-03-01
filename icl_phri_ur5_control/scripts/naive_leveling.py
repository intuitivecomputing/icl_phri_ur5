#!/usr/bin/env python

from __future__ import print_function
import numpy as np
from time import sleep
import sys
from copy import deepcopy

import rospy
import message_filters
from std_msgs.msg import String, Header, Float64
from geometry_msgs.msg import WrenchStamped, Vector3

import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint

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
    

    def set_pose_target(self, value):
        curr_pose = deepcopy(self.get_pose())
        curr_pose.position.z = value
        self._group.set_pose_target(curr_pose)
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
        return self._group.plan()

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

        #self._current_state = self._mg.get_pose().position.z
        #self._state_pub.publish(self._current_state)

            
        wrench_sub = message_filters.Subscriber('icl_phri_gripper/wrench_filtered', 
                                            WrenchStamped)
        control_effort_sub = message_filters.Subscriber('control_effort', Float64)
        ts = message_filters.TimeSynchronizer([wrench_sub, control_effort_sub], 10)
        ts.registerCallback(self._callback)
        print(self._mg.get_pose())
        rospy.spin()
        #self.last_torque = None
        #self.diff = 0
        # self._mg.update_pose_target()
        #r.sleep()

    def _callback(self, wrench, control_effort):
        self._current_state = self._mg.get_pose().position.z
        torque = wrench.wrench.torque.y
        setpoint = 0.0
        print("hahah")
        if torque < 2.0:
            self._state_pub.publish(self._current_state)
            return
        elif torque > 5.0:
            setpoint = self._current_state + 5.0
        else:
            setpoint = self._current_state + (torque-2.0)

        print(torque, self._current_state, setpoint)

        self._mg.set_pose_target(control_effort.data)
        # if not data.data - self._mg.get_pose().position.z < 0.01:
        self._mg.plan_move()

        self._state_pub.publish(self._current_state)
        self._setpoint_pub.publish(setpoint)

        self._mg.go()

    # def _ce_callback(self, data):
    #     self._mg.set_pose_target(data.data)
    #     # if not data.data - self._mg.get_pose().position.z < 0.01:
    #     self._mg.plan_move()
    #     self._mg.go()
        

    # def _wrench_callback(self, msg):
    #     self._current_state = self._mg.get_pose().position.z
    #     torque = msg.wrench.torque.y
    #     setpoint = 0.0
    #     print("hahah")
    #     if torque < 2.0:
    #         self._state_pub.publish(self._current_state)
    #         return
    #     elif torque > 5.0:
    #         setpoint = self._current_state + 5.0
    #     else:
    #         setpoint = self._current_state + (torque-2.0)
    #     print(torque, self._current_state, setpoint)
    #     self._setpoint_pub.publish(setpoint)
    #     self._state_pub.publish(self._current_state)
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

        
if __name__ == '__main__':
    NaiveLeveling()
