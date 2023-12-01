#!/usr/bin/env python


import rospy
from domino_vision_pkg.msg import game_state #service type
import numpy as np

class Conversion:
    def __init__(self):
        rospy.init_node('sub_test', anonymous = True)
        
        self.num_hand_dominos = None
        self.hand_dots_half1 = None
        self.hand_dots_half2 = None
        self.hand_dom_x_cm = None
        self.hand_dom_y_cm = None
        self.hand_dom_orientation = None

        self.num_board_dominos = None
        self.board_dots_half1 = None
        self.board_dots_half2 = None
        self.board_dom_x_cm = None
        self.board_dom_y_cm = None
        self.board_dom_orientation = None 
        

        rospy.Subscriber("/image_info",game_state, self.board_converter)
        rospy.Subscriber("/hand_info",game_state, self.hand_converter)
            

        rospy.spin()


    def hand_converter(self,msg):
        self.num_hand_dominos = msg.num_dominos
        self.hand_dots_half1 = msg.num_dots_half1
        self.hand_dots_half2 = msg.num_dots_half2
        self.hand_dom_x_cm = msg.x
        self.hand_dom_y_cm = msg.y
        self.hand_dom_orientation = msg.orientation

        self.hand_dots_half1 = np.array(self.hand_dots_half1)
        self.hand_dots_half2 = np.array(self.hand_dots_half2)
        
        

    def board_converter(self,msg):
        self.num_board_dominos = msg.num_dominos
        self.board_dots_half1 = msg.num_dots_half1
        self.board_dots_half2 = msg.num_dots_half2
        self.board_dom_x_cm = msg.x
        self.board_dom_y_cm = msg.y
        self.board_dom_orientation = msg.orientation

        self.board_dots_half1 = np.array(self.board_dots_half1)
        self.board_dots_half2 = np.array(self.board_dots_half2)        

    def game_engine(self):
        self.hand_dom_cm = np.vstack((np.array(self.hand_dom_x_cm),np.array(self.hand_dom_y_cm)))
        self.board_dom_cm = np.vstack((np.array(self.board_dom_x_cm),np.array(self.board_dom_y_cm)))

if __name__ == "__main__":
    a = Conversion()
   