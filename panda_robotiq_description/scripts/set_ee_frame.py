#!/usr/bin/env python3
"""Set the robot's end-effector frame to the Robotiq TCP of the URDF.

Every Cartesian quantity the Panda reports or accepts (O_T_EE, O_T_EE_d, O_dP_EE_d, the
end-effector Jacobian, and the Cartesian example controllers built on them) refers to the
end-effector frame configured in the robot:

    F_T_EE = F_T_NE * NE_T_EE

F_T_NE is set in Desk and defaults to the Franka Hand (yaw -45 deg, z 0.1034 m). With the
Robotiq gripper that frame is 45 deg off and 2.7 cm short of the TCP MoveIt plans with.

This script reads F_T_NE from the robot, reads the flange-to-TCP transform
<arm_id>_link8 -> <tcp_link> from TF (so the mount_* and tcp_* arguments of the URDF are
honoured) and calls /franka_control/set_EE_frame with

    NE_T_EE = inv(F_T_NE) * F_T_TCP

Afterwards O_T_EE equals the <tcp_link> pose that MoveIt uses. The mass and centre of mass of
the gripper are not touched; set them in Desk or with /franka_control/set_load.

Run it once after franka_control.launch is up and before a controller is started:

    rosrun panda_robotiq_description set_ee_frame.py

Parameters: ~arm_id (default panda), ~tcp_link (default robotiq_tcp).
"""
import math
import sys

import numpy
import rospy
import tf2_ros
from franka_msgs.msg import FrankaState
from franka_msgs.srv import SetEEFrame
from tf.transformations import euler_from_matrix, quaternion_matrix

STATE_TOPIC = '/franka_state_controller/franka_states'
SERVICE = '/franka_control/set_EE_frame'


def from_column_major(values):
    """16 floats in column-major order (libfranka convention) -> 4x4 matrix."""
    return numpy.array(values).reshape(4, 4).T


def to_column_major(matrix):
    return matrix.T.reshape(16).tolist()


def describe(matrix):
    xyz = matrix[0:3, 3]
    rpy = [math.degrees(a) for a in euler_from_matrix(matrix)]
    return 'xyz (%.4f %.4f %.4f) m  rpy (%.1f %.1f %.1f) deg' % (*xyz, *rpy)


def main():
    rospy.init_node('set_ee_frame')
    arm_id = rospy.get_param('~arm_id', 'panda')
    tcp_link = rospy.get_param('~tcp_link', 'robotiq_tcp')
    flange = '%s_link8' % arm_id

    # Generous timeouts: when started from a launch file, franka_control is still connecting
    try:
        state = rospy.wait_for_message(STATE_TOPIC, FrankaState, timeout=30.0)
    except rospy.ROSException:
        rospy.logerr('No message on %s. Is franka_control running?', STATE_TOPIC)
        sys.exit(1)
    F_T_NE = from_column_major(state.F_T_NE)

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    try:
        t = buf.lookup_transform(flange, tcp_link, rospy.Time(0), rospy.Duration(30.0)).transform
    except tf2_ros.TransformException as e:
        rospy.logerr('No TF %s -> %s. Is robot_description the Robotiq URDF? (%s)', flange, tcp_link, e)
        sys.exit(1)
    F_T_TCP = quaternion_matrix((t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
    F_T_TCP[0:3, 3] = (t.translation.x, t.translation.y, t.translation.z)

    NE_T_EE = numpy.linalg.inv(F_T_NE).dot(F_T_TCP)

    rospy.wait_for_service(SERVICE, timeout=30.0)
    response = rospy.ServiceProxy(SERVICE, SetEEFrame)(NE_T_EE=to_column_major(NE_T_EE))
    if not response.success:
        rospy.logerr('%s failed: %s', SERVICE, response.error)
        sys.exit(1)

    print('F_T_NE  (Desk)           ' + describe(F_T_NE))
    print('F_T_TCP (%s -> %s) ' % (flange, tcp_link) + describe(F_T_TCP))
    print('NE_T_EE (sent)           ' + describe(NE_T_EE))
    print('F_T_EE  (now)            ' + describe(F_T_NE.dot(NE_T_EE)))


if __name__ == '__main__':
    main()
