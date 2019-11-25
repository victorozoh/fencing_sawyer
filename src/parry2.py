#! /usr/bin/env python

import rospy
from intera_interface import Limb
import modern_robotics as mr
import numpy as np
import tf
from geometry_msgs.msg import Transform, Twist, TwistStamped
import sawyer_MR_description as sw


class Parry():

    def __init__(self):
        self.Blist = sw.Blist
        self.M = sw.M
        self.Slist = mr.Adjoint(self.M).dot(self.Blist)
        self.eomg = 0.01 # positive tolerance on end-effector orientation error
        self.ev = 0.001 # positive tolerance on end-effector position error

        self.tf_listener = tf.TransformListener()
        self.arm = Limb()
        self.armpose = None
        self.arm_position = None
        self.arm_angles_dict = {}
        self.end_effector_vel = np.zeros(6)

        self.thetalist0 = []
        self.joint_vels = None
        self.joint_vels_dict = {}
        self.Jacobian = None
        self.pinv_J = None

        self.tracker_x_velocity = 1.0
        self.old_tracker_pose = 0
        self.tracker_pos_mag = 0
        self.tracker_pos = None
        self.tracker_quat = None
        self.pos_diff = 0
        self.displacement = None
        self.disp_mag = None

        # set the end effector twist
        self.arm_twist = Twist()
        self.no_twist = Twist() # to stop arm motion
        self.no_twist.linear.x, self.no_twist.linear.y, self.no_twist.linear.z = 0, 0, 0
        self.no_twist.angular.x, self.no_twist.angular.y, self.no_twist.angular.z = 0, 0, 0


    def twistcallback(self, msg):
        self.tracker_x_velocity = msg.twist.linear.x

    def block(self):
        while not rospy.is_shutdown():
            try:
                # (controller_pos, controller_quat) = listener.lookupTransform('world', 'controller', rospy.Time(0))
                (self.tracker_pos, self.tracker_quat) = self.tf_listener.lookupTransform('world', 'tracker', rospy.Time(0))
                # (arm_position, arm_quat) = listener.lookupTransform('world', 'right_l6', rospy.Time(0))
                self.armpose = self.arm.endpoint_pose()
                self.arm_position = self.armpose['position']
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.tracker_pos = np.array(self.tracker_pos)
            self.arm_position = np.array(self.arm_position)
            self.displacement = self.tracker_pos - self.arm_position

            # set velocity in the direction of the displacement
            self.disp_mag = np.linalg.norm(self.displacement)
            self.tracker_pos_mag = np.linalg.norm(self.tracker_pos)
            print("distance between tracker and world {}".format(self.tracker_pos_mag))
            print("The distance between the arm and tracker is {}".format(self.disp_mag))
            #print("tracker position is {}".format(tracker_pos))
            #print("distance is {}".format(tracker_pos_mag))

            self.arm_twist.linear.x =  0.30 * self.displacement[0]/self.disp_mag
            self.arm_twist.linear.y =  0.30 * self.displacement[1]/self.disp_mag
            self.arm_twist.linear.z =  0.30 * self.displacement[2]/self.disp_mag
            self.arm_twist.angular.x = 0
            self.arm_twist.angular.y = 0
            self.arm_twist.angular.z = 0

            self.pos_diff = np.linalg.norm(self.old_tracker_pose) - self.tracker_pos_mag

            # if user sword is less than 1.25m to robot
            # and distance between robot arm and user sword is less than 0.15m
            # and user sword is moving towards robot...
            if self.tracker_pos_mag < 1.25 and self.disp_mag > 0.15 and self.tracker_x_velocity < 0:
                #pass
                self.move(self.arm_twist)
            else:
                # pass
                self.move(self.no_twist)


            self.old_tracker_pose = self.tracker_pos_mag


    def move(self, twist_msg):
        # have to switch the order of linear and angular velocities in twist
        # message so that it comes in the form needed by the modern_robotics library
        self.end_effector_vel = np.zeros(6)
        self.end_effector_vel[0] = 0 #twist_msg.angular.x
        self.end_effector_vel[1] = 0 #twist_msg.angular.y
        self.end_effector_vel[2] = 0 #twist_msg.angular.z
        self.end_effector_vel[3] = twist_msg.linear.x
        self.end_effector_vel[4] = twist_msg.linear.y
        self.end_effector_vel[5] = twist_msg.linear.z


        self.arm_angles_dict = self.arm.joint_angles()
        self.thetalist0 = [] # initial guess at joint angles
        self.thetalist0.append(self.arm_angles_dict['right_j0'])
        self.thetalist0.append(self.arm_angles_dict['right_j1'])
        self.thetalist0.append(self.arm_angles_dict['right_j2'])
        self.thetalist0.append(self.arm_angles_dict['right_j3'])
        self.thetalist0.append(self.arm_angles_dict['right_j4'])
        self.thetalist0.append(self.arm_angles_dict['right_j5'])
        self.thetalist0.append(self.arm_angles_dict['right_j6'])
        self.thetalist0 = np.array(self.thetalist0)

        self.Jacobian = mr.JacobianSpace(self.Slist, self.thetalist0)
        self.pinv_J = np.linalg.pinv(self.Jacobian)
        # print("The shape of the end effector velocity vector is {}".format(end_effector_vel.shape))
        # print("The shape of the Jacobian Pseudo Inverse matrix is {}".format(pinv_J.shape))
        self.joint_vels = np.dot(self.pinv_J, self.end_effector_vel)
        # velocities need to be passed in as a dictionary
        self.joint_vels_dict = {}

        for i, vel in enumerate(self.joint_vels):
            self.joint_vels_dict['right_j'+ str(i)] = vel

        self.arm.set_joint_velocities(self.joint_vels_dict)


def main():
    rospy.init_node("parry")
    parry = Parry()
    # subscibe to Twist
    twist_sub = rospy.Subscriber('/vive/twist1', TwistStamped, parry.twistcallback, queue_size=1)
    parry.block()
    # rate = rospy.Rate(10.0)
    rospy.spin()

if __name__ == '__main__':
    main()
