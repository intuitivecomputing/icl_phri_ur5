import numpy as np
from time import sleep

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3

from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input  as inputMsg

from robotiq_ctrl import ForceTorqueSensor

if __name__ == '__main__':
    rospy.init_node('average_filter', anonymous=True)