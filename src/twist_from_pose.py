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
arm = None
listener = None
x_vel = np.array([1, 0, 0])
y_vel = np.array([0, 1, 0])
z_vel = np.array([0, 0, 1])

def convert_twist(twist_msg):
    (pos, quat) = listener.lookupTransform('world', 'right_l6', rospy.Time(0))

    R = tf.transformations.quaternion_matrix(quat) # returns a homogenous matrix
    R = np.array(R[:3,:3]) # get just the rotation component

    T_world_to_arm =  mr.RpToTrans(R, pos) # convert to homogenous matrix
    twist_in_arm_frame = np.array([twist_msg.angular.x,
                                   twist_msg.angular.y,
                                   twist_msg.angular.z,
                                   twist_msg.linear.x,
                                   twist_msg.linear.y,
                                   twist_msg.linear.z])
    twist_in_world = mr.Adjoint(T_world_to_arm).dot(twist_in_arm_frame)

    desired_twist = Twist()
    desired_twist.linear.x = twist_in_world[3]
    desired_twist.linear.y = twist_in_world[4]
    desired_twist.linear.z = twist_in_world[5]
    desired_twist.angular.x = twist_in_world[0]
    desired_twist.angular.y = twist_in_world[1]
    desired_twist.angular.z = twist_in_world[2]
    return desired_twist



def move(twist_msg):
    # have to switch the order of linear and angular velocities in twist
    # message so that it comes in the form needed by the modern_robotics library
    end_effector_vel = np.zeros(6)
    end_effector_vel[0] = 0 #twist_msg.angular.x
    end_effector_vel[1] = 0 #twist_msg.angular.y
    end_effector_vel[2] = 0 #twist_msg.angular.z
    end_effector_vel[3] = twist_msg.linear.x
    end_effector_vel[4] = twist_msg.linear.y
    end_effector_vel[5] = twist_msg.linear.z


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

    J = mr.JacobianSpace(Slist, thetalist0)
    pinv_J = np.linalg.pinv(J)
    # print("The shape of the end effector velocity vector is {}".format(end_effector_vel.shape))
    # print("The shape of the Jacobian Pseudo Inverse matrix is {}".format(pinv_J.shape))
    joint_vels = np.dot(pinv_J,end_effector_vel)
    # velocities need to be passed in as a dictionary
    joint_vels_dict = {}

    for i, vel in enumerate(joint_vels):
        joint_vels_dict['right_j'+ str(i)] = vel

    arm.set_joint_velocities(joint_vels_dict)


def main():
    rospy.init_node("twist_from_pose")
    global arm
    global listener
    arm = Limb()
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (pos, quat) = listener.lookupTransform('right_l6', 'controller', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # set the end effector twist
        arm_twist = Twist()
        pos = np.array(pos)
        linear_vel = 0.02 * (pos/np.linalg.norm(pos))
        arm_twist.linear.x =  np.linalg.norm(x_vel * np.dot(linear_vel, x_vel)/np.dot(x_vel, x_vel))
        arm_twist.linear.y =  np.linalg.norm(y_vel * np.dot(linear_vel, y_vel)/np.dot(y_vel, y_vel))
        arm_twist.linear.z =  np.linalg.norm(z_vel * np.dot(linear_vel, z_vel)/np.dot(z_vel, z_vel))
        arm_twist.angular.x = 0
        arm_twist.angular.y = 0
        arm_twist.angular.z = 0
        # need to transform into a twist in the world frame
        arm_twist = convert_twist(arm_twist)

        move(arm_twist)

        rate.sleep()

if __name__ == '__main__':
    main()
