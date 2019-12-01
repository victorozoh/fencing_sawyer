#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion
from intera_interface import Limb

home_config = {'right_j6': -1.3186796875, 'right_j5': 0.5414912109375,
               'right_j4': 2.9682451171875, 'right_j3': 1.7662939453125,
               'right_j2': -3.0350302734375, 'right_j1': 1.1202939453125, 'right_j0': -0.0001572265625}

def main():

    rospy.init_node("get_arm_pose")
    arm = Limb()
    tf_listener = tf.TransformListener()

    armpose = arm.endpoint_pose()
    quat_arr = armpose['orientation']
    print("position is {}".format(armpose['position']))
    print("\n")
    print("orientation is {}".format(quat_arr))
    # euler_angles = euler_from_quaternion(quat_arr, 'sxyz')
    # print("The euler angles are {}".format(euler_angles))


    #joints = arm.joint_angles()
    #print(joints)
    # rate = rospy.Rate(10.0)
    # while not rospy.is_shutdown():
    #     try:
    #         # (controller_pos, controller_quat) = listener.lookupTransform('world', 'controller', rospy.Time(0))
    #         (pos, quat) = tf_listener.lookupTransform('world', 'reference/right_hand', rospy.Time(0))
    #         print("right hand orientation is {}".format(euler_from_quaternion(quat, 'sxyz')))
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    #     rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    main()
