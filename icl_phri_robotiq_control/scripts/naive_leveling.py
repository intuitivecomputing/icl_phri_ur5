#!/usr/bin/env python

from __future__ import print_function
import numpy as np
from time import sleep
import sys
import copy

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3

from utils import *

class MoveGroup:
    def __init__(self):
        print("============ Starting tutorial setup")
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface',
        #                 anonymous=True)
        ## Instantiate a RobotCommander object.  This object is an interface to
        ## the robot as a whole.
        self._robot = moveit_commander.RobotCommander()

        ## Instantiate a PlanningSceneInterface object.  This object is an interface
        ## to the world surrounding the robot.
        self._scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a MoveGroupCommander object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the left
        ## arm.  This interface can be used to plan and execute motions on the left
        ## arm.
        self._group = moveit_commander.MoveGroupCommander("manipulator")

        self.pose_target = geometry_msgs.msg.Pose()
        ## We create this DisplayTrajectory publisher which is used below to publish
        ## trajectories for RVIZ to visualize.
        self._display_trajectory_publisher = rospy.Publisher(
                                            '/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory, 
                                            queue_size=1)

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

    def update_pose_target(self):
        self.pose_target = self.get_pose()

    def plan_move(self):
        ## Planning to a Pose goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the 
        ## end-effector
        print("============ Generating plan 1")
        ## Now, we call the planner to compute the plan
        ## and visualize it if successful
        ## Note that we are just planning, not asking move_group 
        ## to actually move the robot
        #self._group.set_pose_target(self.pose_target)
        self.plan = self._group.plan()

        # print("============ Waiting while RVIZ displays plan1...")
        # rospy.sleep(5)

        # ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
        # ## group.plan() method does this automatically so this is not that useful
        # ## here (it just displays the same trajectory again).
        # print("============ Visualizing plan1")
        # display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        # display_trajectory.trajectory_start = self.robot.get_current_state()
        # display_trajectory.trajectory.append(self.plan)
        # self.display_trajectory_publisher.publish(display_trajectory)

        # print("============ Waiting while plan1 is visualized (again)...")
        # rospy.sleep(5)

        # Moving to a pose goal
        # ^^^^^^^^^^^^^^^^^^^^^
        #
        # Moving to a pose goal is similar to the step above
        # except we now use the go() function. Note that
        # the pose goal we had set earlier is still active 
        # and so the robot will try to move to that goal. We will
        # not use that function in this tutorial since it is 
        # a blocking function and requires a controller to be active
        # and report success on execution of a trajectory.

        # Uncomment below line when working with a real robot

    def go(self):
        self._group.go(wait=True)

    def __del__(self):
        moveit_commander.roscpp_shutdown()


class NaiveLeveling:
    def __init__(self):
        rospy.init_node('naive_leveling', anonymous=True)
        r = rospy.Rate(15)
        self._mg = MoveGroup()
        self._wrench_sub = rospy.Subscriber('icl_phri_gripper/wrench_filtered', WrenchStamped, self._wrench_callback, queue_size=1)
        print(self._mg.get_pose())
        self.last_torque = None
        self.diff = 0
        self._mg.update_pose_target()
        #r.sleep()
        rospy.spin()

    def _wrench_callback(self, msg):
        if self.last_torque:
            self.diff = msg.wrench.torque.y - self.last_torque
            self.last_torque = msg.wrench.torque.y
        else:
            self.last_torque = msg.wrench.torque.y
            self.diff = 0
        if self.diff > 0.1:
            self._mg.shift_pose_target('z', 0.005)
            self._mg.plan_move()
            self._mg.go()
        elif self.diff < -0.1:
            self._mg.shift_pose_target('z', -0.005)
            self._mg.plan_move()
            self._mg.go()
        print(self.diff)  
        print(self._mg.pose_target.position) 


        
if __name__ == '__main__':
    NaiveLeveling()
