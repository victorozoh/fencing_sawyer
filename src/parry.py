#! /usr/bin/env python

import rospy
from intera_interface import Limb
import modern_robotics as mr
import numpy as np
import tf
from geometry_msgs.msg import Transform, Twist, TwistStamped
import sawyer_MR_description as sw

Blist = sw.Blist
M = sw.M
Slist = mr.Adjoint(M).dot(Blist)
eomg = 0.01 # positive tolerance on end-effector orientation error
ev = 0.001 # positive tolerance on end-effector position error
arm = None
listener = None
home_config = {'right_j6': -1.3186796875, 'right_j5': 0.5414912109375,
               'right_j4': 2.9682451171875, 'right_j3': 1.7662939453125,
               'right_j2': -3.0350302734375, 'right_j1': 1.1202939453125, 'right_j0': -0.0001572265625}


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
    rospy.init_node("parry")
    global arm
    global listener
    arm = Limb()
    listener = tf.TransformListener()
    old_tracker_pose = 0
    # set the end effector twist
    arm_twist = Twist()
    no_twist = Twist()
    no_twist.linear.x, no_twist.linear.y, no_twist.linear.z = 0, 0, 0
    no_twist.angular.x, no_twist.angular.y, no_twist.angular.z = 0, 0, 0

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # (controller_pos, controller_quat) = listener.lookupTransform('world', 'controller', rospy.Time(0))
            (tracker_pos, tracker_quat) = listener.lookupTransform('world', 'tracker', rospy.Time(0))
            # (arm_position, arm_quat) = listener.lookupTransform('world', 'right_l6', rospy.Time(0))
            armpose = arm.endpoint_pose()
            arm_position = armpose['position']
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        tracker_pos = np.array(tracker_pos)
        arm_position = np.array(arm_position)
        displacement = tracker_pos - arm_position

        # set velocity in the direction of the displacement
        disp_mag = np.linalg.norm(displacement)
        tracker_pos_mag = np.linalg.norm(tracker_pos)
        print("distance between tracker and world {}".format(tracker_pos_mag))
        print("The distance between the arm and tracker is {}".format(disp_mag))
        #print("tracker position is {}".format(tracker_pos))
        #print("distance is {}".format(tracker_pos_mag))

        arm_twist.linear.x =  0.20 * displacement[0]/disp_mag
        arm_twist.linear.y =  0.20 * displacement[1]/disp_mag
        arm_twist.linear.z =  0.20 * displacement[2]/disp_mag
        arm_twist.angular.x = 0
        arm_twist.angular.y = 0
        arm_twist.angular.z = 0

        pos_diff = np.linalg.norm(old_tracker_pose) - tracker_pos_mag

        # if user sword is less than 1.25m to robot
        # and distance between robot arm and user sword is less than 0.15m
        # and user sword is moving towards robot...
        if tracker_pos_mag < 1.25 and disp_mag > 0.15:
            #pass
            move(arm_twist)
        elif tracker_pos_mag > 1.25:
            #pass
            #move(no_twist)
            arm.set_joint_positions(home_config)
        else:
            move(no_twist)


        old_tracker_pose = tracker_pos_mag
        rate.sleep()

if __name__ == '__main__':
    main()
