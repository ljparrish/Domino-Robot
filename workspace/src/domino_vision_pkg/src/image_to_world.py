#!/usr/bin/env python

import rospy
import numpy as np
from domino_vision_pkg.msg import game_state
from domino_vision_pkg.msg import world_coordinates

class Image_to_world:
    def __init__(self):
        rospy.init_node('image_to_world', anonymous = True)

        self.fx = 625.95398
        self.fy = 624.580994
        self.ox = 363.980011
        self.oy = 208.332001
        self.dom_sub = rospy.Subscriber("/board_info", game_state, self.image_to_world)
        rospy.spin()


    def pixel_to_point(self,u,v):
        depth = 4
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
        X,Y,Z = self.pixel_to_point(u,v)
        self.world_pub = rospy.Publisher('/world_coord',world_coordinates, queue_size = 10)
        r = rospy.Rate(10)      
        pub_string = world_coordinates(x = X, y = Y, z = Z)
        self.world_pub.publish(pub_string)
        r.sleep()

if __name__ == '__main__':
    while not rospy.is_shutdown(): 
        Image_to_world()
