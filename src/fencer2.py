#! /usr/bin/env python

import rospy
from intera_interface import Limb
import modern_robotics as mr
import numpy as np
import tf
import time
from geometry_msgs.msg import Transform, Twist, TwistStamped
from std_msgs.msg import Float64
import sawyer_MR_description as sw


class Fencer():

    def __init__(self):
        self.Blist = sw.Blist
        self.M = sw.M
        self.Slist = mr.Adjoint(self.M).dot(self.Blist)
        self.eomg = 0.01 # positive tolerance on end-effector orientation error
        self.ev = 0.001 # positive tolerance on end-effector position error
        self.arm = None
        self.listener = None
        self.home_config = {'right_j6': -1.3186796875, 'right_j5': 0.5414912109375,
                       'right_j4': 2.9682451171875, 'right_j3': 1.7662939453125,
                       'right_j2': -3.0350302734375, 'right_j1': 1.1202939453125, 'right_j0': -0.0001572265625}

        self.tracker_position = None
        self.tracker_pos_mag = None
        self.arm_position = None
        self.sword_position = None
        self.attack_position = None
        self.tag_position = None
        self.disp_mag = None
        self.attack_disp_mag = None
        self.fence_region = 1.50
        self.fence_speed = 0.50
        self.sword_zoffset = 0.25
        self.end_effector_directions = [-1.0, 1.0]
        self.tracker_twist = 50
        self.end_effector_vel = np.zeros(6)
        # velocities need to be passed in as a dictionary
        self.joint_vels_dict = {}

        # set the end effector twist
        self.sword_twist = Twist()
        self.arm_twist = Twist()
        self.no_twist = Twist()
        self.no_twist.linear.x = 0
        self.no_twist.linear.y = 0
        self.no_twist.linear.z = 0
        self.no_twist.angular.x = 0
        self.no_twist.angular.y = 0
        self.no_twist.angular.z = 0

        self.arm = Limb()
        print("going to home configuration")
        self.arm.set_joint_positions(self.home_config)
        self.clash_pub = rospy.Publisher('clash', Float64, queue_size=1)
        self.twist_sub = rospy.Subscriber('/vive/twist1', TwistStamped, self.twist_callback, queue_size=1)
        self.tf_listener = tf.TransformListener()

        self.rate = rospy.Rate(10.0)


    def move(self, twist_msg):
        # have to switch the order of linear and angular velocities in twist
        # message so that it comes in the form needed by the modern_robotics library

        self.end_effector_vel[0] = 0 #twist_msg.angular.x
        self.end_effector_vel[1] = 0 #twist_msg.angular.y
        self.end_effector_vel[2] = 0 #twist_msg.angular.z
        self.end_effector_vel[3] = twist_msg.linear.x
        self.end_effector_vel[4] = twist_msg.linear.y
        self.end_effector_vel[5] = twist_msg.linear.z

        arm_angles_dict = self.arm.joint_angles()
        thetalist0 = []
        for i in range(len(arm_angles_dict)):
            thetalist0.append(arm_angles_dict['right_j'+ str(i)])

        J = mr.JacobianSpace(self.Slist, np.array(thetalist0))
        pinv_J = np.linalg.pinv(J)
        # print("The shape of the end effector velocity vector is {}".format(self.end_effector_vel.shape))
        # print("The shape of the Jacobian Pseudo Inverse matrix is {}".format(pinv_J.shape))
        joint_vels = np.dot(pinv_J,self.end_effector_vel)

        for i, vel in enumerate(joint_vels):
            self.joint_vels_dict['right_j'+ str(i)] = vel
        # set joint velocities
        self.arm.set_joint_velocities(self.joint_vels_dict)


    def twist_callback(self, msg):
        # get magnitude of tracker twist
        self.tracker_twist = np.linalg.norm([msg.twist.linear.x, \
                                             msg.twist.linear.y, \
                                             msg.twist.linear.z])

    def fence(self):

        while not rospy.is_shutdown():
            try:
                # (controller_pos, controller_quat) = listener.lookupTransform('world', 'controller', rospy.Time(0))
                (self.tracker_position, _) = self.tf_listener.lookupTransform('world', 'tracker', rospy.Time(0))
                # get sword pose
                (self.sword_position, _) = self.tf_listener.lookupTransform('world', 'sword_tip', rospy.Time(0))
                # get tag position
                # (self.tag_position, _) = self.tf_listener.lookupTransform('world', 'ar_marker_0', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.tracker_position = None
                self.sword_position = None
                self.tag_position = None
                self.arm.set_joint_positions(self.home_config)
                continue

            # get arm position
            armpose = self.arm.endpoint_pose()
            self.arm_position = armpose['position']

            self.tracker_position = np.array(self.tracker_position)
            self.arm_position = np.array(self.arm_position)
            self.sword_position = np.array(self.sword_position)
            self.tag_position = np.array(self.tag_position)
            #offset tracker position so that robot sword meets at mid point
            self.tracker_position[2] += self.sword_zoffset

            print("tracker twist is {}".format(self.tracker_twist))
            if self.tracker_twist > 0.20: # user moving sword
                self.defend()
            else:
                self.attack()

            self.rate.sleep()


    def defend(self):
        if self.tracker_position is None:
            self.arm.set_joint_positions(self.home_config)
            return
        else:
            displacement = self.tracker_position - self.sword_position
        # set velocity in the direction of the displacement
        self.disp_mag = np.linalg.norm(displacement)
        self.clash_pub.publish(self.disp_mag)
        self.tracker_pos_mag = np.linalg.norm(self.tracker_position)
        # print("distance between tracker and world {}".format(tracker_pos_mag))
        self.arm_twist.linear.x =  self.fence_speed * displacement[0]/self.disp_mag
        self.arm_twist.linear.y =  self.fence_speed * displacement[1]/self.disp_mag
        self.arm_twist.linear.z =  self.fence_speed * displacement[2]/self.disp_mag
        self.arm_twist.angular.x = self.fence_speed * np.random.choice(self.end_effector_directions)
        self.arm_twist.angular.y = 0
        self.arm_twist.angular.z = 0

        if self.tracker_pos_mag < self.fence_region and self.disp_mag > 0.07:
            #pass
            self.move(self.arm_twist)
            if self.disp_mag < 0.40:
                self.arm.set_joint_positions(self.arm.joint_angles())
                time.sleep(1)
        elif self.tracker_pos_mag > self.fence_region:
            self.arm.set_joint_positions(self.home_config)
        else:
            #pass
            self.move(self.no_twist)


    def attack(self):
        #self.arm.set_joint_positions(self.home_config)
        # define new attack point
        x_point = np.random.uniform(low=self.tracker_position[0]+0.10 ,high=self.tracker_position[0]+0.10)
        y_point = np.random.uniform(low=self.tracker_position[1]-0.50 ,high=self.tracker_position[1])
        z_point = np.random.uniform(low=self.tracker_position[2] ,high=self.tracker_position[2]+0.40)

        self.attack_position = np.array([x_point, y_point, z_point])
        # if self.tag_position is None:
        #     self.arm.set_joint_positions(self.home_config)
        #     return
        # else:
        #     displacement = self.tag_position - self.arm_position
        displacement = self.attack_position - self.arm_position
        # set velocity in the direction of the displacement
        self.attack_disp_mag = np.linalg.norm(displacement)
        self.clash_pub.publish(self.disp_mag)

        self.arm_twist.linear.x =  self.fence_speed * displacement[0]/self.attack_disp_mag
        self.arm_twist.linear.y =  self.fence_speed * displacement[1]/self.attack_disp_mag
        self.arm_twist.linear.z =  self.fence_speed * displacement[2]/self.attack_disp_mag
        self.arm_twist.angular.x = 0
        self.arm_twist.angular.y = self.fence_speed * np.random.choice(self.end_effector_directions)
        self.arm_twist.angular.z = 0

        self.move(self.arm_twist)
        if self.disp_mag < 0.40:
            self.arm.set_joint_positions(self.arm.joint_angles())
            time.sleep(1)
        elif self.tracker_pos_mag > self.fence_region:
            self.arm.set_joint_positions(self.home_config)
        else:
            #pass
            #self.move(self.no_twist)
            self.arm.set_joint_positions(self.home_config)



if __name__ == '__main__':
    rospy.init_node("fencer")
    # initialize fence object
    fencer = Fencer()
    fencer.fence()
