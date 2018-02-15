import numpy as np

from geometry_msgs.msg import WrenchStamped, Vector3, Wrench

def vector3_to_numpy(vect):
    return np.array([vect.x, vect.y, vect.z])

def array_to_wrench(array):
    msg = Wrench()
    msg.wrench.force.x = array[0]
    msg.wrench.force.y = array[1]
    msg.wrench.force.z = array[2]
    msg.wrench.torque.x = array[3]
    msg.wrench.torque.y = array[4]
    msg.wrench.torque.z = array[5]
    return msg

def array_to_wrench_stamped(header, array):
    msg = WrenchStamped()
    msg.header = header
    msg.wrench = array_to_wrench(array)
    return msg