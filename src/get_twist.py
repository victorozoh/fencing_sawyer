#! /usr/bin/env python

import rospy
import tf
import numpy as np
import modern_robotics as mr
import geometry_msgs.msg

def twistCallback(msg):
    twist_msg = msg.twist

    # (pos,quat) = listener.lookupTransform('world', 'world_vive', rospy.Time(0))
    R = tf.transformations.quaternion_matrix(quat) # returns a homogenous matrix
    R = np.array(R[:3,:3]) # get just the rotation component

    T_world_to_worldvive =  mr.RpToTrans(R, pos) # convert to homogenous matrix
    twist_in_worldvive = np.array([twist_msg.angular.x,
                                   twist_msg.angular.y,
                                   twist_msg.angular.z,
                                   twist_msg.linear.x,
                                   twist_msg.linear.y,
                                   twist_msg.linear.z])
    twist_in_world = mr.Adjoint(T_world_to_worldvive).dot(twist_in_worldvive)

    # set twist values
    vive_twist = geometry_msgs.msg.Twist()
    vive_twist.linear.x = twist_in_world[3]
    vive_twist.linear.y = twist_in_world[4]
    vive_twist.linear.z = twist_in_world[5]
    vive_twist.angular.x = twist_in_world[0]
    vive_twist.angular.y = twist_in_world[1]
    vive_twist.angular.z = twist_in_world[2]

    # publish the vive controller twist
    vive_twist_pub.publish(vive_twist)
    # else:
    #     temp = geometry_msgs.msg.Twist()
    #     temp.linear.x, temp.linear.y, temp.linear.z = 0, 0, 0
    #     temp.angular.x, temp.angular.y, temp.angular.z = 0, 0, 0
    #     vive_twist_pub.publish(temp)


def main():
    # global listener
    global vive_twist_pub
    global pos
    global quat
    rospy.init_node('get_twist')
    # This approach tries to get sawyer arm to follow the vive controller twist
    # listener = tf.TransformListener()
    # transform between world and world_vive is fixed. Set in vive.launch file in vive_ros package
    pos = rospy.get_param("/vive/world_offset")
    pos = np.array(pos)
    quat = [0.000, 0.000, 0.000, 0.000]#rospy.get_param("/vive/world_yaw")

    # subscribe to velocities published by vive_node
    sub = rospy.Subscriber('/vive/twist1', geometry_msgs.msg.TwistStamped, twistCallback, queue_size=1)
    vive_twist_pub = rospy.Publisher('vive_twist', geometry_msgs.msg.Twist,queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
