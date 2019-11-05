#! /usr/bin/env python

import rospy
import tf
import geometry_msgs.msg

def main():
    rospy.init_node('get_pose')

    # This approach tries to get sawyer arm to get the current position of the
    # Vive controller
    listener = tf.TransformListener()
    vive_pose_pub = rospy.Publisher('vive_pose', geometry_msgs.msg.Transform, queue_size=1)
    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown():
        try:
            (trans, quat) = listener.lookupTransform('world', 'controller', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        vive_pose = geometry_msgs.msg.Transform()
        vive_pose.translation.x = trans[0]
        vive_pose.translation.y = trans[1]
        vive_pose.translation.z = trans[2]

        vive_pose.rotation.x = 0 #quat[0]
        vive_pose.rotation.y = 0 #quat[1]
        vive_pose.rotation.z = 0 #quat[2]
        vive_pose.rotation.w = 0 #quat[3]
        # publish the vive controller pose
        vive_pose_pub.publish(vive_pose)
        rate.sleep()

if __name__ == '__main__':
    main()
