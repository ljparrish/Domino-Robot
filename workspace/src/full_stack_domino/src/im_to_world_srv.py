#!/usr/bin/env python

import rospy
import numpy as np
from domino_vision_pkg.srv import position_state_srv
import tf 
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


class Image_to_world():
    def __init__(self,service):
        # Store Service Name
        self.serviceName = service

        # Camera Matrix Properties
        self.fx = 628.270996 #pixels
        self.fy = 627.080017 #pixels
        self.depth = 0.29 #meters

        self.ox = 357.515991
        self.oy = 216.927002
        
        self.im2world_service = rospy.Service(service, position_state_srv, self.image_to_world)

        # Adjustment Error for Image2World
        self.x_offset = 0
        self.y_offset = 0
        
        self.tf_listener = tf.TransformListener()
        

    def setOffset(self,x,y):
        try:
            self.x_offset = x
            self.y_offset = y
            return True
        except:
            return False

    def setDepth(self,newDepth):
        self.depth = newDepth

    def pixel_to_point(self,u,v):
        camera_X = np.zeros(np.size(u))
        camera_Y = np.zeros(np.size(u))
        camera_Z = np.zeros(np.size(u))
        world_X = np.zeros(np.size(u))
        world_Y = np.zeros(np.size(u))
        world_Z = np.zeros(np.size(u))
        for i in range(np.size(u)):
            camera_X[i] = (u[i]-self.ox)*self.depth/self.fx
            camera_Y[i] = (v[i]-self.oy)*self.depth/self.fy
            camera_Z[i] = self.depth 
            try: 
                self.tf_listener.waitForTransform("/base","/right_hand_camera", rospy.Time(),rospy.Duration(10.0))
                world_point = self.tf_listener.transformPoint("base", PointStamped(header=Header(stamp=rospy.Time(),frame_id="right_hand_camera"), point=Point(camera_X[i],camera_Y[i],camera_Z[i])))
                world_X[i], world_Y[i], world_Z[i] = world_point.point.x, world_point.point.y, world_point.point.z
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("TF Error")
                return
        return world_X + self.x_offset, world_Y + self.y_offset, world_Z
    def image_to_world(self, msg): 
        u = msg.x
        u = np.array(u)
        v = msg.y
        v = np.array(v)
        X,Y,Z = self.pixel_to_point(u,v)
        print(X)
        print(Y)
        print(Z)
        print(msg.num_dots_half1)
        print(msg.num_dots_half2)
        print(msg.orientation)
        print(f"Service Callback Triggered for {self.serviceName} at {rospy.get_time()}")
        return list(X), list(Y), list(Z), msg.orientation, msg.num_dots_half1, msg.num_dots_half2

if __name__ == '__main__':
    rospy.init_node('image_to_world', anonymous = True)
 
    # Sets up 2 instances of the Image to world class, one for the robots hand, one for the game board
    Board = Image_to_world("image_to_board")
    Hand = Image_to_world("image_to_hand")
    Hand.setDepth(0.3206)
    rospy.spin()