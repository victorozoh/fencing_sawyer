#! /usr/bin/env python

import vlc
import rospy
from geometry_msgs.msg import TwistStamped


class SoundPlayer():
    def __init__(self):
        self.player = vlc.MediaPlayer("/home/victor/sawyerws/src/fencing_sawyer/assets/light-saber-battle.mp3")
        self.twist_sub = rospy.Subscriber('/vive/twist1', TwistStamped, self.playsound, queue_size=1)

    def playsound(self, msg):
        if msg.twist.linear.x < 0:
            self.player.play()
        else:
            self.player.stop()
            # pass

def main():
    rospy.init_node("sound_player")
    # create sound player
    sound_player = SoundPlayer()
    rospy.spin()

if __name__ == "__main__":
    main()
