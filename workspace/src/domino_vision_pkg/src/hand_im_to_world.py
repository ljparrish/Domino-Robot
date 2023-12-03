#!/usr/bin/env python

import rospy
import numpy as np
import sys
import os
sys.path.append(os.path.abspath("/Domino-Robot/workspace/src/domino_vision_pkg"))
from msg import image_info, position_state
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import tf 
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


class Image_to_world():
    def __init__(self,topic):

        self.fx = 628.270996 #pixels
        self.fy = 627.080017 #pixels
        self.depth = 0.29 #meters

        self.ox = 357.515991
        self.oy = 216.927002
        self.dom_sub = rospy.Subscriber("/image_info", image_info, self.image_to_world)
        self.world_pub = rospy.Publisher(topic, position_state, queue_size = 10)

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
        num_dots1 = msg.num_dots_half1
        num_dots2 = msg.num_dots_half2
        orientation = msg.orientation
        X,Y,Z = self.pixel_to_point(u,v)
        pub_string = position_state(x = X, y = Y, z = Z, num_dots_half1 = num_dots1, num_dots_half2 = num_dots2, orientation = orientation)
        self.world_pub.publish(pub_string)

if __name__ == '__main__':
    rospy.init_node('image_to_world', anonymous = True)
 
    ImageToWorldTester = Image_to_world()

