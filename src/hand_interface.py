# HELPER functions for the arm interface
from geometry_msgs.msg import (
    Twist,
    Pose
    )
from intera_interface import Limb
import rospy

def move_to_home(thetalist):
    rightlimb = Limb()
    waypoints = {}
    waypoints['right_j0'] = thetalist[0]
    waypoints['right_j1'] = thetalist[1]
    waypoints['right_j2'] = thetalist[2]
    waypoints['right_j3'] = thetalist[3]
    waypoints['right_j4'] = thetalist[4]
    waypoints['right_j5'] = thetalist[5]
    waypoints['right_j6'] = thetalist[6]
    waypoints['torso_t0'] = thetalist[7]
    rightlimb.move_to_joint_positions(waypoints, timeout = 20.0, threshold = 0.05)
    rospy.sleep(2.0)

def publishTwist(publisher, veltwist, boundxhit=0, boundzhit=0):

    if boundxhit == 1:
        # set linear velocities
        veltwist.linear.x = -1*abs(veltwist.linear.x)

    if boundxhit == 2:
        # set linear velocities
        veltwist.linear.x = abs(veltwist.linear.x)

    if boundzhit == 1:
        # set linear velocities
        veltwist.linear.z = -1*abs(veltwist.linear.z)

    if boundzhit == 2:
        # set linear velocities
        veltwist.linear.z = abs(veltwist.linear.z)
    # publish the twist
    publisher.publish(veltwist)
