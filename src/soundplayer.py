#! /usr/bin/env python

from pydub import AudioSegment
from pydub.playback import play

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64


class SoundPlayer():
    def __init__(self):
        self.saber_on = AudioSegment.from_mp3("/home/victor/sawyerws/src/fencing_sawyer/assets/SaberOn.mp3")
        play(self.saber_on)
        self.clash_sound = AudioSegment.from_mp3("/home/victor/sawyerws/src/fencing_sawyer/assets/clash2.mp3")
        #self.twist_sub = rospy.Subscriber('/vive/twist1', TwistStamped, self.playsound, queue_size=1)
        self.sub = rospy.Subscriber('clash', Float64, self.clash_callback, queue_size=1)


    def clash_callback(self, msg):
        if msg.data < 0.07:
            play(self.clash_sound)



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
