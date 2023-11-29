#! /usr/bin/env python
import rospy
import cv2
import numpy as np
import tf
from domino_vision_pkg.msg import game_state

class Image_to_world:
   def __init__(self):
       rospy.init_node('image_to_world', anonymous = True)

       self.fx = None
       self.fy = None
       self.ox = None
       self.oy = None
       
       self.dom_sub = rospy.Subscriber("/board_info", game_state, image_to_world)
       self.tf_Listener = tf.Transformlistener()
       rospy.spin()
   
   def camera_info_callback(self, msg):
       self.fx = 625.95398
       self.fy = 624.580994
       self.ox = 363.980011
       self.oy = 208.332001
   def pixel_to_point(self,u,v,depth):
       world_X = (u-self.ox)*depth/self.fx
       world_Y = (v-self.oy)*depth/self.fy
       world_Z = depth 
       return world_X, world_Y, world_Z


def image_to_world(message):
    print("Initializing node... ")
    #rospy.init_node('domino_detection', anonymous = True)
    #rospy.Service('domino_detection', game_state, callback)
    #rospy.loginfo('Running domino_detection')
    #rospy.spin()
    image_x = message.x
    image_y = message.y
    

    depth = ...

    
    
    # subscribes to the topic that publishes current position
    # Write an if statement to see when certain positions are reached
    pub_board = rospy.Publisher('/board_info', game_state, queue_size=10)
    #pub_hand = rospy.Publisher('/hand_info', game_state, queue_size = 10)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        
        num_dominos, num_dots1, num_dots2, x_cm, y_cm, orientation = domino_visualization()
        pub_string = game_state(num_dominos = num_dominos, num_dots_half1 = num_dots1, num_dots_half2 = num_dots2, x = x_cm, y = y_cm, orientation = orientation)
        pub_board.publish(pub_string)
        r.sleep()


if __name__ == '__main__':
   Image_to_world()
