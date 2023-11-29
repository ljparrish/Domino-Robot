#! /usr/bin/env python


import rospy
from domino_vision_pkg.msg import game_state #service type
import numpy as np

yes = np.array([80,100,100])
def callback(message):
    num_dominos = message.num_dominos
    num_dots_half1 = message.num_dots_half1
    if x == 100 and y == 100 and z == 100:
        print(num_dominos)
        print(num_dots_half1)

def subscriber():
    rospy.Subscriber("/board_info", game_state, callback)
    rospy.spin()


if __name__ == '__main__':
    x = yes[0]
    y = yes[1]
    z = yes[2]
    if x == 100 and y == 100 and z == 100:
        rospy.init_node('subscriber', anonymous = True)
        subscriber()