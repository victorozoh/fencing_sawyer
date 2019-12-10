#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf

rospy.init_node("track_tag")


def tag_callback(msg):
    pose = msg.pose.position
    alvartag_pub.publish(pose)



# alvartag_pub = rospy.Publisher('/viz_marker_pose', Point, queue_size=1)
# alvartag_sub = rospy.Subscriber('/visualization_marker', Marker, tag_callback, queue_size=1)

tf_listener = tf.TransformListener()

#rospy.spin()
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        (tag_position, _) = tf_listener.lookupTransform('world', 'ar_marker_0', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    print("tag is at position {}".format(tag_position))
    rate.sleep()
