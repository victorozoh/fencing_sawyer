#! /usr/bin/env python

import rospy
from intera_interface import HeadDisplay

def main():
    rospy.init_node("display_head")

    head_display = HeadDisplay()
    image_file_path = ["/home/victor/sawyerws/src/fencing_sawyer/assets/grievous_battlefront.jpg"]
    head_display.display_image(image_file_path, True, 1.0)


if __name__ == "__main__":
    main()
