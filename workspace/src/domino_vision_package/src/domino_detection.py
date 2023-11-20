import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy
#from domino_vision_package import game_state #service type

def image_capture():
    cam = cv2.VideoCapture(0) 
    s, im = cam.read()
    #cv2.imshow("Test", im)
    cv2.imwrite("test.jpg", im)

    image = im      
    with open("rect_test.py") as f:
        exec(f.read())
    #result, image = cam.read()
    #cv2.imshow("snapshot", image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
image_capture()


def domino_detection():
    rospy.init_node('domino_detection')
    rospy.Service('domino_detection',game_state,image_capture)

if __name__ == '__main__':

  domino_detection()
  

  