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
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
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

##from ur_kinematics.ur_kin_py import forward, inverse
# from ur_kinematics.ur_kin_py import forward, inverse
from icl_phri_robotiq_control.robotiq_utils import *

normalize = lambda x: x/np.sqrt(x[0]**2.+x[1]**2.+x[2]**2.)
norm = lambda a:np.sqrt(a.x**2.+a.y**2.+a.z**2.)
to_quat = lambda o: np.array([o.x, o.y, o.z, o.w])
rad_to_ang = lambda x: x / np.pi * 180.
ang_to_rad = lambda x: x / 180. * np.pi
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [0.09981511533260345, -2.024665657673971, -0.7203519980060022, -1.7111480871783655, 1.5250813961029053, 0.0007902098586782813]
Q2 = [0.09786205738782883, -2.2906211058246058, -0.7209880987750452, -1.749324146901266, 1.5192002058029175, 0.001340735238045454]
Q3 = [-1.725508991871969, -0.9326947371112269, -1.830911938344137, -0.3809617201434534, 1.7116585969924927, 0]
# Q_fetch = [1.5279078483581543, -2.149566952382223, -1.1292513052569788, 0.14056265354156494, 1.786577582359314, 1.541101336479187]
# Q_wait = [0.6767016649246216, -1.2560237089740198, -1.9550021330462855, -0.019442383443013966, 1.5472047328948975, 1.5413050651550293]
# Q_give = [-0.20308286348451787, -2.2909210363971155, -1.2152870337115687, 0.34171295166015625, 1.4318565130233765, 1.5419158935546875]

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
#             ]from sensor_msgs.msg import JointState
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
        self.client = actionlib.SimpleActionClient('icl_phri_ur5/follow_joint_trajectory', FollowJointTrajectoryAction)
        print ("Waiting for server...")
        self.client.wait_for_server()
        
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
        # print('waypount', self.waypoints)
        (cartesian_plan, fraction) = self._group.compute_cartesian_path(
                                     self.waypoints,   # waypoints to follow
                                     0.01,        # eef_step, 1cm
                                     0.0)         # jump_threshold, disabling
        rospy.sleep(2)
        self._group.execute(cartesian_plan)
        self.waypoints.pop(0)

    def cart_move(self, Q):
        self.append_waypoint(Q)

    def joint_move(self, qs):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        move_time = 3.0
        rospy.sleep(1)
        try:
            joint_states = rospy.wait_for_message("icl_phri_ur5/joint_states", JointState)
            joints_pos = joint_states.position
            #joints_pos = self._group.get_current_joint_values()
            g.trajectory.points = []
            time_from_start = 0.0
            g.trajectory.points.append(JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(time_from_start)))
            for q in qs:
                time_from_start = time_from_start + move_time
                g.trajectory.points.append(JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(time_from_start)))
            self.client.send_goal(g)
            return self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except e:
            print(e)
        
        
    # def get_pose(self):
    #     return self._group.get_current_pose().pose

    # def shift_pose_target(self, axis, value):
    #     axis_dict = {'x': 0, 'y': 1, 'z': 2, 'r': 3, 'p': 4, 'y': 5}
    #     print('ee: %s' % self._group.get_end_effector_link())
    #     self._group.shift_pose_target(axis_dict[axis], value, self._group.get_end_effector_link())
    #     #self.pose_target = self.get_pose()

class Handover:
    def __init__(self):
        self._mg = MoveGroup()
        self.handover = True
        self.fetch = False
        self._gripper_ac = RobotiqActionClient('icl_phri_gripper/gripper_controller')
        self._gripper_ac.wait_for_server()
        print('============Init gripper============')
        self._gripper_ac.initiate()
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        # self.ik = InverseKinematics()
        self._mg.joint_move([Q1, Q2])
        self._gripper_ac.send_goal(0)
        rospy.sleep(2)
        print('33')
        self._mg.joint_move([Q1, Q3])
        print('33')

#-----------------get two frame transformation------------------#
    def _get_transform(self, target_frame, source_frame):       
        try:
            (trans, rot) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("wis_handover = Falseait for tf")

   
    def _list_add(self, ls1, ls2):
        return [x + y for x, y in zip(ls1, ls2)]


    def _get_handover_pos(self):
        (trans1, rot1) = self._get_transform('/base_link', '/right_hand_1')
        offset = ([0, -0.35, 0],
                            [0, 0, 0, 0])
        # rot_new = tf.transformations.quaternion_multiply(rot1, offset[1])
        #tf.transformations.quaternion_from_euler()
        trans_new = self._list_add(trans1, offset[0])
        # Q = trans_new + list(rot_new)
        rot_new = [-0.707, 0, 0, 0.707]
        Q = trans_new + list(rot_new)
        # ps = PoseStamped()
        # ps.header.frame_id = "ee_link"
        # ps.pose.position = Point(*trans_new)
        # ps.pose.orientation = Quaternion(*rot_new) 
        # args = ["manipulator", "ee_link", ps]
        # print('hhhhhhhhh',self.ik.getIK(*args, attempts=1))
        # rospy.loginfo( str( ))
        self.broadcaster.sendTransform(
            tuple(trans_new),
            tuple(rot_new),
            rospy.Time.now(),
            'target',
            'base_link'
        )
        return Q

    def frame_action(self, trans):
        if trans[0] > 0.85 and self.handover:
            rospy.sleep(1)
            Q_new = self._get_handover_pos()
            print('Q_new', Q_new)
            self._mg.cart_move(Q_new)
            rospy.sleep(2)
            self._gripper_ac.send_goal(0.14)
            self.handover = False
            self.fetch = True
            rospy.sleep(2)
        elif self.fetch:
            rospy.sleep(1)
            self._mg.joint_move([Q3])
        

            # self.fetch = True
        # if abs(trans[0]) < 0.4 and self.fetch:
        #     print(self._mg.move([Q_wait]))
        #     self.fetch = False


if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    h = Handover()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        try:
            # rospy.sleep(1)
            trans = h._get_transform('/left_hand_1', '/right_hand_1')[0]
            # print("hand_gripper_tf_trans:{}\nhand_gripper_tf_quat:{}".format(
            #     h._get_transform('/ee_link', '/right_hand_1')[0],
            #     h._get_transform('/ee_link', '/right_hand_1')[1]
            # ))
            # print(trans)
            h.frame_action(trans)
        except e:
            print(e)

# from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse,\
#                             GetPositionIK, GetPositionIKRequest, GetPositionIKResponse,\
#                             GetStateValidity, GetStateValidityRequest, GetStateValidityResponse
# DEFAULT_FK_SERVICE = "/compute_fk"
# DEFAULT_IK_SERVICE = "/compute_ik"
# DEFAULT_SV_SERVICE = "/check_state_validity"
# class InverseKinematics():
#     """Simplified interface to ask for inverse kinematics"""
#     def __init__(self):
#         rospy.loginfo("Loading InverseKinematics class.")
#         self.ik_srv = rospy.ServiceProxy(DEFAULT_IK_SERVICE, GetPositionIK)
#         rospy.loginfo("Connecting to IK service")
#         self.ik_srv.wait_for_service()
#         rospy.loginfo("Ready for making IK calls")

#     def closeIK(self):
#         self.ik_srv.close()
        
#     def getIK(self, group_name, ik_link_name, pose, avoid_collisions=True, attempts=None, robot_state=None, constraints=None):
#         """Get the inverse kinematics for a group with a link a in pose in 3d world.
#         @group_name string group i.e. right_arm that will perform the IK
#         @ik_link_name string link that will be in the pose given to evaluate the IK
#         @pose PoseStamped that represents the pose (with frame_id!) of the link
#         @avoid_collisions Bool if we want solutions with collision avoidance
#         @attempts Int number of attempts to get an Ik as it can fail depending on what IK is being used
#         @robot_state RobotState the robot state where to start searching IK from (optional, current pose will be used
#         if ignored)"""
#         gpikr = GetPositionIKRequest()
#         gpikr.ik_request.group_name = group_name
#         if robot_state != None:  # current robot state will be used internally otherwise
#             gpikr.ik_request.robot_state = robot_state
#         gpikr.ik_request.avoid_collisions = avoid_collisions
#         gpikr.ik_request.ik_link_name = ik_link_name
#         if type(pose) == type(PoseStamped()):
#             gpikr.ik_request.pose_stamped = pose
#         else:
#             rospy.logerr("pose is not a PoseStamped, it's: " + str(type(pose)) + ", can't ask for an IK")
#             return
#         if attempts != None:
#             gpikr.ik_request.attempts = attempts
#         else:
#             gpikr.ik_request.attempts = 0
#         if constraints != None:
#             gpikr.ik_request.constraints = constraints
#         ik_result = self.ik_srv.call(gpikr)
#         rospy.logwarn("Sent: " + str(gpikr))
#         return ik_result
