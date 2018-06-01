#!/usr/bin/env python

from __future__ import division, print_function
import numpy as np
from math import *
from time import sleep
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import actionlib
import rospy
from ros_myo.msg import EmgArray
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3
import tf
from tf.transformations import *
# import roslib
# roslib.load_manifest("ur_kinematics")
# import sys
#import ur_kin_py
##from ur_kinematics.ur_kin_py import forward, inverse
from ur_kinematics.ur_kin_py import forward, inverse
from icl_phri_robotiq_control.robotiq_utils import *

normalize = lambda x: x/np.sqrt(x[0]**2.+x[1]**2.+x[2]**2.)
norm = lambda a:np.sqrt(a.x**2.+a.y**2.+a.z**2.)
to_quat = lambda o: np.array([o.x, o.y, o.z, o.w])
rad_to_ang = lambda x: x / np.pi * 180.
ang_to_rad = lambda x: x / 180. * np.pi
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joi        self.joints_pos_start[3] = self.joints_pos_start[3]+diff_znt', 'wrist_3_joint']
Q_fetch = [1.5279078483581543, -2.149566952382223, -1.1292513052569788, 0.14056265354156494, 1.786577582359314, 1.541101336479187]
Q_wait = [0.6767016649246216, -1.2560237089740198, -1.9550021330462855, -0.019442383443013966, 1.5472047328948975, 1.5413050651550293]
Q_give = [-0.20308286348451787, -2.2909210363971155, -1.2152870337115687, 0.34171295166015625, 1.4318565130233765, 1.5419158935546875]

# def best_sol(sols, q_guess, weights):
#     valid_sols = []
#     for sol in sols:
#         test_sol = np.ones(6)*9999.
#         for i in range(6):
#             for add_ang in [-2.*np.pi, 0, 2.*np.pi]:
#                 test_ang = sol[i] + add_ang
#                 if (abs(test_ang) <= 2.*np.pi and 
#                     abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
#                     test_sol[i] = test_ang
#         if np.all(test_sol != 9999.):
#             valid_sols.append(test_sol)
#     if len(valid_sols) == 0:
#         return None
#     best_sol_ind = np.argmin(np.sum((weights*(valid_sols - np.array(q_guess)))**2,1))
#     return valid_sols[best_sol_ind]

# class Handover:
#     def __init__(self):
#         #/vel_based_pos_traj_controller/
#         self.client = actionlib.SimpleActionClient('icl_phri_ur5/follow_joint_trajectory', FollowJointTrajectoryAction)
#         self.goal = FollowJointTrajectoryGoal()
#         self.goal.trajectory = JointTrajectory()
#         self.goal.trajectory.joint_names = JOINT_NAMES
#         print ("Waiting for server...")
#         self.client.wait_for_server()
#         print ("Connected to server")
#         joint_states = rospy.wait_for_message("icl_phri_ur5/joint_states", JointState)
#         self.joints_pos_start = np.array(joint_states.position)
#         print ("Init done")
#         self.listener = tf.TransformListener()

#     def move(self):
#         current_m = forward(self.joints_pos_start)
#         hand_base = self._get_transform('/ee_link', '/right_hand_1')
#         gripper_hand = None
#         Desti_m = self._cal_dest(gripper_hand, hand_base)
#         sols = inverseinverse(np.array(Desti_m), float(self.joints_pos_start[5]))
#         qsol = best_sol(sols, self.joints_pos_start, [1.]*6)
#         if qsol is not None:
#             self.goal.trajectory.points = [
#                 JointTrajectoryPoint(positions=self.joints_pos_start.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
#                 JointTrajectoryPoint(positions=qsol.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(1./125.)),
#             ]
#             print('start: ' + str(self.joints_pos_start.tolist()))
#             print('goal: ' + str(qsol.tolist()))
#             try:
#                 self.client.send_goal(self.goal)
#                 self.joints_pos_start = qsol
#             except:
#                 raise
    
#     def _get_transform(self, target_frame, source_frame, is_matrix=True):
#         rate = rospy.Rate(10.0)
#         rate.sleep()        
#         # try:
#         (trans, quat) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
#         if is_matrix:
#             rot = euler_from_quaternion(quat)            
#             return np.asarray(tf.TransformerROS.fromTranslationRotation(trans, rot))
#         else:
#             return trans, quat
#         # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#         #     print("wait for tf")
    
#     def _cal_dest(self, gripper_hand, hand_base):
#         return np.dot(gripper_hand , hand_base)
        





class MoveGroup:
    def __init__(self):
        print ("============ Starting Moveit setup==============")
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('move_group_python_interface',
                  #anonymous=True)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("manipulator")
        display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)
        
        print ("============ Reference frame: %s" % self._group.get_planning_frame())
        print ("============ End effector: %s" % self._group.get_end_effector_link())
        print ("============ Robot Groups:")
        print (self._robot.get_group_names())
        print ("============ Printing robot state")
        print (self._robot.get_current_state())
        print ("============")
        self.waypoints = []
        self.waypoints.append(self._group.get_current_pose().pose) # start with the current pose
        self.wpose = geometry_msgs.msg.Pose()
        
    def append_waypoint(self, Q):
        self.wpose = copy.deepcopy(self._group.get_current_pose().pose)
        #self.wpose.position.z = self._group.get_current_pose().pose.position.z + diff
        self.wpose.position.x = Q[0]
        self.wpose.position.y = Q[1]
        self.wpose.position.z = Q[2]
        self.wpose.orientation.x = Q[3]
        self.wpose.orientation.y = Q[4]
        self.wpose.orientation.z = Q[5]
        self.wpose.orientation.w = Q[6]
        self.waypoints.append(copy.deepcopy(self.wpose))
        print('waypount', self.waypoints)
        (cartesian_plan, fraction) = self._group.compute_cartesian_path(
                                     self.waypoints,   # waypoints to follow
                                     0.005,        # eef_step, 1cm
                                     0.0)         # jump_threshold, disabling
        rospy.sleep(1)
        self._group.execute(cartesian_plan)
        self.waypoints.pop(0)

    def move(self, Q):
        self.append_waypoint(Q)
        
        
    def get_pose(self):
        return self._group.get_current_pose().pose

    def shift_pose_target(self, axis, value):
        axis_dict = {'x': 0, 'y': 1, 'z': 2, 'r': 3, 'p': 4, 'y': 5}
        print('ee: %s' % self._group.get_end_effector_link())
        self._group.shift_pose_target(axis_dict[axis], value, self._group.get_end_effector_link())
        #self.pose_target = self.get_pose()

class Handover:
    def __init__(self):
        self._mg = MoveGroup()
        self._gripper_ac = RobotiqActionClient('icl_phri_gripper/gripper_controller')
        self._gripper_ac.wait_for_server()
        print('============Init gripper============')
        self._gripper_ac.initiate()
        self.listener = tf.TransformListener()
        self.fetch = False

#-----------------get two frame transformation------------------#
    def _get_transform(self, target_frame, source_frame):
        rate = rospy.Rate(10.0)
        rate.sleep()        
        try:
            (trans, rot) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("wait for tf")

   
    def _list_add(self, ls1, ls2):
        return [x + y for x, y in zip(ls1, ls2)]


    def _get_handover_pos(self):
        (trans1, rot1) = self._get_transform('/right_hand_1', '/base_link')
        offset = ([0, -0.2, 0.2],
                            [0, 0, 0, 0])
        # rot_new = tf.transformations.quaternion_multiply(rot1, offset[1])
        #tf.transformations.quaternion_from_euler()
        trans_new = self._list_add(trans1, offset[0])
        # Q = trans_new + list(rot_new)
        rot_new = [0, 0, 0.707, 0.707]
        Q = trans_new + list(rot_new)
        return Q
        

    def frame_action(self, trans):
        if abs(trans[0]) > 0.4:
            rospy.sleep(1)
            Q_new = self._get_handover_pos()
            print('Q_new', Q_new)
            self._mg.move(Q_new)
            # self.fetch = True
        # if abs(trans[0]) < 0.4 and self.fetch:
        #     print(self._mg.move([Q_wait]))
        #     self.fetch = False

if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    h = Handover()
    while not rospy.is_shutdown():
        try:
            rate = rospy.Rate(10.0)
            rate.sleep()
            trans = h._get_transform('/right_hand_1', '/right_shoulder_1')[0]
            # print("hand_gripper_tf_trans:{}\nhand_gripper_tf_quat:{}".format(
            #     h._get_transform('/ee_link', '/right_hand_1')[0],
            #     h._get_transform('/ee_link', '/right_hand_1')[1]
            # ))
            h.frame_action(trans)
        except:
            print('no action')
