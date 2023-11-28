#! /usr/bin/env python


import rospy
from domino_vision_pkg.srv import camera_take_pic #service type

def game_planner():
    rospy.init_node('game_planner_client')
    rospy.wait_for_service('camera_display')
    try:
        proxy = rospy.ServiceProxy('camera_display',camera_take_pic)
        x = 100
        y = 100
        z = 100
        proxy(x,y,z)
    except rospy.ServiceException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    game_planner()