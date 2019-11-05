#! /usr/bin/env python

import rospy
from intera_interface import Limb
import modern_robotics as mr
import numpy as np
import tf
from geometry_msgs.msg import Transform, Twist
# get Slist from Jarvis description file
import sawyer_MR_description as sw

Blist = sw.Blist
M = sw.M
Slist = mr.Adjoint(M).dot(Blist)
eomg = 0.01 # positive tolerance on end-effector orientation error
ev = 0.001 # positive tolerance on end-effector position error

def move(vive_twist):

    # have to switch the order of linear and angular velocities in twist
    # message so that it comes in the form needed by the modern_robotics library
    end_effector_vel = np.zeros(6)
    end_effector_vel[0] = vive_twist.angular.x
    end_effector_vel[1] = vive_twist.angular.y
    end_effector_vel[2] = vive_twist.angular.z
    end_effector_vel[3] = vive_twist.linear.x
    end_effector_vel[4] = vive_twist.linear.y
    end_effector_vel[5] = vive_twist.linear.z

    arm = Limb()
    arm_angles_dict = arm.joint_angles()
    thetalist0 = [] # initial guess at joint angles
    thetalist0.append(arm_angles_dict['right_j0'])
    thetalist0.append(arm_angles_dict['right_j1'])
    thetalist0.append(arm_angles_dict['right_j2'])
    thetalist0.append(arm_angles_dict['right_j3'])
    thetalist0.append(arm_angles_dict['right_j4'])
    thetalist0.append(arm_angles_dict['right_j5'])
    thetalist0.append(arm_angles_dict['right_j6'])
    thetalist0 = np.array(thetalist0)

    J = mr.JacobianSpace(Slist, thetalist)
    pinv_J = np.linalg.pinv(J)
    # print("The shape of the end effector velocity vector is {}".format(end_effector_vel.shape))
    # print("The shape of the Jacobian Pseudo Inverse matrix is {}".format(pinv_J.shape))
    joint_vels = np.dot(pinv_J,end_effector_vel)
    # velocities need to be passed in as a dictionary
    joint_vels_dict = {}

    for i in range(len(joint_vels)):
        joint_vels_dict['right_j'+ str(i)] = joint_vels[i]

    limb.set_joint_velocities(joint_vels_dict)

def main():
    try:
        rospy.init_node("vive_tracker_twist")
        # subscribe to twists published by get_twist node
        sub = rospy.Subscriber('vive_twist', Twist, move, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not perform the requested motion.')

if __name__ == '__main__':
    main()
