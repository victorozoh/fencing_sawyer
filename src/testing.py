#!/usr/bin/env python

import rospy

from intera_interface import Limb
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

# Project-specific imports
import hand_interface as hif

thetalistHOME = [1.3395771484375, -3.5355, 2.0365224609375, -1.489580078125, -0.4218515625, 1.1975029296875, -3.419748046875, 0.0]

def main():
    #print(dir(Limb)) # works
    # print(dir(MotionTrajectory))
    try:
        rospy.init_node("armcontrol")
        arm = Limb()
        armpose = arm.endpoint_pose()
        print("position is {}".format(armpose['position']))
        print("orientation is {}".format(armpose['orientation']))
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not perform the requested motion.')



if __name__ == "__main__":
    main()
