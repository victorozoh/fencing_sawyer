#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import (
    TrajectoryOptions
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose
)

from tf_conversions import posemath
from intera_interface import Limb
import modern_robotics as mr

# get Slist from Jarvis description file
import sawyer_MR_description as sw

# get end effector velocity from pong logic
def move(velocity):
    global limb
    global callback_pub
    # have to switch the order of linear and angular velocities in twist
    # message so that it comes in the form needed by the modern_robotics library
    end_effector_vel = np.zeros(6)
    end_effector_vel[0] = velocity.angular.x
    end_effector_vel[1] = velocity.angular.y
    end_effector_vel[2] = velocity.angular.z
    end_effector_vel[3] = velocity.linear.x
    end_effector_vel[4] = velocity.linear.y
    end_effector_vel[5] = velocity.linear.z
    # we'll treat the velocity vector as a twist 6x1
    # 1. Compute the Jacobian. Get the Blist and thetalist
    limb = Limb()
    thetadict = limb.joint_angles()
    rospy.loginfo(thetadict)
    thetalist = []

    Blist = sw.Blist
    M = sw.M
    Slist = mr.Adjoint(M).dot(Blist)
    thetalist.append(thetadict['right_j0'])
    thetalist.append(thetadict['right_j1'])
    thetalist.append(thetadict['right_j2'])
    thetalist.append(thetadict['right_j3'])
    thetalist.append(thetadict['right_j4'])
    thetalist.append(thetadict['right_j5'])
    thetalist.append(thetadict['right_j6'])
    thetalist = np.array(thetalist)
    J = mr.JacobianSpace(Slist, thetalist)
    pinv_J = np.linalg.pinv(J)
    print("The shape of the end effector velocity vector is {}".format(end_effector_vel.shape))
    print("The shape of the Jacobian Pseudo Inverse matrix is {}".format(pinv_J.shape))
    joint_vels = np.dot(pinv_J,end_effector_vel)
    # velocities need to be passed in as a dictionary
    joint_vels_dict = {}

    for i in range(len(joint_vels)):
        joint_vels_dict['right_j'+ str(i)] = joint_vels[i]

    limb.set_joint_velocities(joint_vels_dict)
    endpoint_Pose = limb.endpoint_pose()

    currPose = Pose()
    currPose.position = endpoint_Pose['position']
    currPose.orientation = endpoint_Pose['orientation']

    callback_pub.publish(currPose)

def main():
    global limb
    global callback_pub
    try:
        rospy.init_node("armcontrol")
        callback_pub = rospy.Publisher("endpoint_Pose", Pose, queue_size=1)
        limb = Limb()
        # subscribe to pong velocities published by pong node
        sub = rospy.Subscriber('pongvelocity', Twist, move, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not perform the requested motion.')

if __name__=="__main__":
    main()
