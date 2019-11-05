#! /usr/bin/env python

import rospy
from intera_interface import Limb
import modern_robotics as mr
import numpy as np
import tf
from geometry_msgs.msg import Transform
# get Slist from Jarvis description file
import sawyer_MR_description as sw

Blist = sw.Blist
M = sw.M
Slist = mr.Adjoint(M).dot(Blist)
eomg = 0.01 # positive tolerance on end-effector orientation error
ev = 0.001 # positive tolerance on end-effector position error

def move(vive_pose):
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

    trans = np.array([vive_pose.translation.x, vive_pose.translation.y, vive_pose.translation.z])
    quat = [vive_pose.rotation.x, vive_pose.rotation.y, vive_pose.rotation.z, vive_pose.rotation.w]

    R = tf.transformations.quaternion_matrix(quat) # returns a homogenous matrix
    R = np.array(R[:3,:3]) # get just the rotation component

    desired_pose =  mr.RpToTrans(R, trans) # convert to homogenous matrix

    # Compute inverse kinematics as described in Modern Robotics text by Kevin Lynch
    [thetalist, success] = mr.IKinSpace(Slist, M, desired_pose, thetalist0, eomg, ev)

    joint_pos_dict = {}
    for i, theta in enumerate(thetalist):
        joint_pos_dict['right_j'+ str(i)] = theta

    if success:
        arm.set_joint_positions(joint_pos_dict)

def main():
    try:
        rospy.init_node("vive_tracker_pose")
        # subscribe to vive poses published by get_pose node
        sub = rospy.Subscriber('vive_pose', Transform, move, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not perform the requested motion.')

if __name__ == '__main__':
    main()
