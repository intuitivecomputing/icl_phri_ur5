#!/usr/bin/env python
from __future__ import division, print_function
import numpy as np
from time import sleep


import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3

from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg

from utils import vector3_to_numpy


class RobotiqGripperCtrl:
    def __init__(self):
        # input gACT gGTO gSTA gOBJ gFLT gPR gPO gCU
        self.query = inputMsg.CModel_robot_input()
        # self.activated = False
        self.current = 0.0
        self.requested_pos = 0
        self.curr_pos = 0
        # gOBJ: Only valid if gGTO = 1.
        # 0x00 - Fingers are in motion towards requested position. No object detected.
        # 0x01 - Fingers have stopped due to a contact while opening before requested position. Object detected.
        # 0x02 - Fingers have stopped due to a contact while closing before requested position. Object detected.
        # 0x03 - Fingers are at requested position. No object detected or object has been lost / dropped.
        self.grasp_status = 0
        # output_msg rACT rGTO rATR rPR rSP rFR
        self.command = outputMsg.CModel_robot_output()
        self.query_sub = rospy.Subscriber(
            'CModelRobotInput', inputMsg.CModel_robot_input, self.query_callback)
        self.command_pub = rospy.Publisher(
            'CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=1)
        self.reset()
        sleep(1)
        self.activate()
        sleep(3)

    def query_callback(self, input):
        print("callback")
        print(self.query.gSTA)
        self.query = input
        self.activated = True if self.query.gSTA == 1 else False

        if self.query.gSTA == 1:                          # activaton in progress
            print("Activation in progress")
        elif not self.query.gSTA == 3:                    # not activated
            print("Not activated. Try to activate.")
            self.activate()
        else:                                           # activated
            #self.activated = True
            self.grasp_status = self.query.gOBJ
            self.requested_pos = self.query.gPR
            self.curr_pos = self.query.gPO

    def reset(self):
        print('reset')
        self.command = outputMsg.CModel_robot_output()
        self.command.rACT = 0
        self.command_pub.publish(self.command)

    def activate(self):
        # if not self.activated:
        print('activate')
        self.command = outputMsg.CModel_robot_output()
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP = 255
        self.command.rFR = 150
        self.command_pub.publish(self.command)
        self.activated = True

    def close_gripper(self, position=255):
        self.command.rPR = position
        self.send_command()

    def open_gripper(self):
        self.command.rPR = 0
        self.send_command()

    def set_speed(self, speed=150):
        self.command.rSP = speed
        # self.command_pub.publish(command)

    def set_force(self, force=127):
        self.command.rFR = force
        # self.command_pub.publish(command)

    def send_command(self):
        if self.command.rACT == 1:
            self.command_pub.publish(self.command)
        else:
            print("Not Activated.")

    def is_grasp_success(self):
        print("Grasp status: " + str(self.grasp_status))
        if self.grasp_status == 2:
            return True
        else:
            return False


def norm(vect):
    return (vect.x**2.0 + vect.y**2.0 + vect.z**2.0)**0.5

# class MyVector3:
#     def __init__(self, vect):
#         self.x = vect.x
#         self.y = vect.y
#         self.z = vect.z
#         self.mag = norm(vect)
#         self.dir = np.arrary([vect.x, vect.y, vect.z]) / self.mag



class ForceTorqueSensor:
    def __init__(self, topic_name):
        self.fts_sub = rospy.Subscriber(
            topic_name, WrenchStamped, self.fts_callback)
        self.header = Header()
        self.force = np.zeros(3)
        self.torque = np.zeros(3)

    def fts_callback(self, wrench_stamped):
        self.header = wrench_stamped.header
        self.force = vector3_to_numpy(wrench_stamped.wrench.force)
        self.torque = vector3_to_numpy(wrench_stamped.wrench.torque)

    def get_force(self):
        return self.force

    def get_torque(self):
        return self.torque

    # def get_force_mag(self):
    #     return self.force.mag

    # def get_force_dir(self):
    #     return self.force.dir


if __name__ == '__main__':
    rospy.init_node('robotiq_ctrl', anonymous=True)
    gripper_ctrl = RobotiqGripperCtrl()
    force_torque_sensor = ForceTorqueSensor('robotiq_force_torque_wrench')
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        print(force_torque_sensor.get_force())
        if force_torque_sensor.get_force()[2] < -25.0:
            gripper_ctrl.set_speed(150)
            gripper_ctrl.close_gripper()
            print ('Closing gripper')
        if force_torque_sensor.get_force()[2] > -20.0:
            gripper_ctrl.set_speed(150)
            print('curr_pos:' + str(gripper_ctrl.curr_pos))
            gripper_ctrl.close_gripper(
                0 if (gripper_ctrl.curr_pos < 5) else gripper_ctrl.curr_pos - 5)
            sleep(0.5)
            gripper_ctrl.close_gripper(
                0 if (gripper_ctrl.curr_pos < 10) else gripper_ctrl.curr_pos - 10)
            gripper_ctrl.open_gripper()
            print('Opening gripper')
        rate.sleep()
