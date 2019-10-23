#!/usr/bin/env python

from intera_interface import Limb
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

def main():
    #print(dir(Limb)) # works
    print(dir(MotionTrajectory))


if __name__ == "__main__":
    main()
