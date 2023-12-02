#!/usr/bin/env python

import rospy
import numpy as np
from domino_vision_pkg.msg import image_info
from domino_vision_pkg.msg import position_state
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import tf 
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


class Image_to_world:
    def __init__(self):
        rospy.init_node('image_to_world', anonymous = True)

        self.fx = 625.95398 #pixels
        self.fy = 624.580994 #pixels
        self.depth = 0.45

        # Change to lower left corner
        self.ox = 0.0
        self.oy = 0
        self.dom_sub = rospy.Subscriber("/image_info", image_info, self.image_to_world)
        
        self.tf_listener = tf.TransformListener()
        
        rospy.spin()

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
                world_point = self.tf_listener.transformPoint("game_board", PointStamped(header=Header(stamp=rospy.Time(),frame_id="right_hand_camera"), point=Point(camera_X[i],camera_Y[i],camera_Z[i])))
                world_X[i], world_Y[i], world_Z[i] = world_point.point.x, world_point.point.y, world_point.point.z
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("TF Error: " + e)
                return
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
        print(X, Y, Z)
        self.world_pub = rospy.Publisher('/board_info',position_state, queue_size = 10)
        r = rospy.Rate(10)      
        pub_string = position_state(x = X, y = Y, z = Z, num_dots_half1 = num_dots1, num_dots_half2 = num_dots2, orientation = orientation)
        self.world_pub.publish(pub_string)
        r.sleep()

if __name__ == '__main__':
    while not rospy.is_shutdown(): 
        Image_to_world()
