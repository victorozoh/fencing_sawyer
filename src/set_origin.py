#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

def main():
    try:
        rospy.init_node("set_origin")

        # wait for service to be advertised
        rospy.wait_for_service('/vive/set_origin')

        client = rospy.ServiceProxy('/vive/set_origin', Empty)

        client()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not set the origin.')

if __name__ == '__main__':
    main()
