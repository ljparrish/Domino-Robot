#!/usr/bin/env python

import rospy
import numpy as np
from domino_vision_pkg.msg import image_info
from domino_vision_pkg.msg import position_state

class Image_to_world:
    def __init__(self):
        rospy.init_node('image_to_world', anonymous = True)

        self.fx = 625.95398 #pixels
        self.fy = 624.580994 #pixels
        self.ox = 0.0
        self.oy = 0.0
        self.dom_sub = rospy.Subscriber("/image_info", image_info, self.image_to_world)
        rospy.spin()


    def pixel_to_point(self,u,v):
        depth =  450 # mm
        world_X = np.zeros(np.size(u))
        world_Y = np.zeros(np.size(u))
        world_Z = np.zeros(np.size(u))

        for i in range(np.size(u)):
            world_X[i] = (u[i]-self.ox)*depth/self.fx
            world_Y[i] = (v[i]-self.oy)*depth/self.fy
            world_Z[i] = depth 
        return world_X, world_Y, world_Z
    def image_to_world(self, msg): 
        u = msg.x
        u = np.array(u)
        v = msg.y
        v = np.array(v)
        num_dots1 = msg.num_dots_half1
        num_dots2 = msg.num_dots_half2
        orientation = msg.orientation
        X,Y,Z = self.pixel_to_point(u,v)
        self.world_pub = rospy.Publisher('/board_info',position_state, queue_size = 10)
        r = rospy.Rate(10)      
        pub_string = position_state(x = X, y = Y, z = Z, num_dots_half1 = num_dots1, num_dots_half2 = num_dots2, orientation = orientation)
        self.world_pub.publish(pub_string)
        r.sleep()

if __name__ == '__main__':
    while not rospy.is_shutdown(): 
        Image_to_world()
