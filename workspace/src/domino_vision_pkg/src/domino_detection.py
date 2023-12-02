#!/usr/bin/env python

import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy
import os

from domino_vision_pkg.msg import image_info #msg type

def domino_visualization():
    path = os.getcwd()
    path = path + '/src/domino_vision_pkg/src'
    img = cv2.imread(os.path.join(path, 'test2.jpg'))
    #img = cv2.resize(img, None, fx = 0.5, fy = 0.5)
    #img = cv2.GaussianBlur(img,(5,5),0)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # Convert to grayscale
    ret,thresh = cv2.threshold(gray,127,255,0) # Apply black/white mask. TUNE THIS BASED ON LIGHTING CONDITIONS Ada: 210-220ish, Alan: 127
    #cv2.imshow("Shapes", thresh)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    contours,hierarchy = cv2.findContours(thresh, 1, 2)
    #print("Number of contours detected:", len(contours))

    # Initialize blob detector:
    params = cv2.SimpleBlobDetector_Params()
    params.filterByCircularity = True
    #params.minCircularity = 0.7
    #params.maxCircularity = 1
    detector = cv2.SimpleBlobDetector_create(params)

    # Initialize Empty things we want to send
    num_dominos = 0
    num_dots1 = []
    num_dots2 = []
    x_cm = []
    y_cm = []
    orientation = []
    xcmh = []
    ycmh = []

    # Iterating over contours detected to determine if it is a rectangle 
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True) #use 0.03 if we want to see the skinny rectangle
        if len(approx) == 4:
            #x, y, w, h = cv2.boundingRect(cnt)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            ((x,y),(w,h),angle) = rect

            #If working on ROS/Lab computers, use angle < -45. If using laptop, use angle > 45
            if angle < -45:
                swapped = rect[1]
                h = swapped[0]
                w = swapped[1]
                
            #print('x:',x)
            #print('y:',y)
            #print('w:',w)
            #print('h:',h)
            ratio = float(w)/h # Rectangle's width to height ratio
            
            if w > h: # Assume horizontal orientation for domino
                if ratio > 1.9 and ratio < 2.1: # 5 is the maximum ratio to prevent seeing the middle black line 
                    

                    # Create a cropped image to count just the dots in that domino, splitting it in half.
                    crop1 = thresh[int(y-h//2):int(y+h//2), int(x-w//2):int(x)] 
                    crop2 = thresh[int(y-h//2):int(y+h//2), int(x):int(x+w//2)]
                    

                    # Dots:
                    keypoints1 = detector.detect(crop1)
                    #print("Black Dots Count for half 1/2:",len(keypoints1))
                    num_dots1.append(len(keypoints1))
                    #cv2.imshow("Cropped", crop1)
                    #cv2.waitKey(0)
                    #cv2.destroyAllWindows()
                    
                    keypoints2 = detector.detect(crop2)
                    #print("Black Dots Count for half 2/2:",len(keypoints2))
                    num_dots2.append(len(keypoints2))
                    #cv2.imshow("Cropped", crop2)
                    #cv2.waitKey(0)
                    #cv2.destroyAllWindows()

                    # Outline Rectangle:
                    #cv2.putText(img, 'Rectangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    #img = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)
                    img = cv2.drawContours(img, [box], -1, (255,0,0), 3)

                    # COM + Orientation + num_dots + num_dom
                    x_cm.append(x)
                    y_cm.append(y)
                    orientation.append('H')
                    num_dots1.append(len(keypoints1))
                    num_dots2.append(len(keypoints2))
                    num_dominos = num_dominos + 1

                    # Domino half center of mass for horizontal orientation:
                    xcm_h1 = x - w//4
                    ycm_h1 = y
                    xcm_h2 = x + w//4
                    ycm_h2 = y
                    #print('cm_h1:',xcm_h1,ycm_h1)
                    #print('cm_h2:',xcm_h2,ycm_h2)
                    xcmh.append(xcm_h1)
                    xcmh.append(xcm_h2)
                    ycmh.append(ycm_h1)
                    ycmh.append(ycm_h2)

                    

            elif h > w: # assume domino is in vertical orientation 
                if ratio > 0.4 and ratio < 0.6: #0.2 is the minimum ratio to prevent seeing the middle black line 
                    # Create a cropped image to count just the dots in that domino. 
                    crop1 = thresh[int(y-h//2):int(y), int(x-w//2):int(x+w//2)] 
                    crop2 = thresh[int(y):int(y+h//2), int(x-w//2):int(x+w//2)]

                    # Dots:
                    keypoints1 = detector.detect(crop1)
                    #print("Black Dots Count for half 1/2:",len(keypoints1))
                    num_dots1.append(len(keypoints1))
                    #cv2.imshow("Cropped", crop1)
                    #cv2.waitKey(0)
                    #cv2.destroyAllWindows()
                    
                    keypoints2 = detector.detect(crop2)
                    #print("Black Dots Count for half 2/2:",len(keypoints2))
                    num_dots2.append(len(keypoints2))
                    #cv2.imshow("Cropped", crop2)
                    #cv2.waitKey(0)
                    #cv2.destroyAllWindows()
                    

                    # Outline Rectangle:
                    #cv2.putText(img, 'Rectangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    #img = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)
                    img = cv2.drawContours(img, [box], -1, (0,0,255), 3)

                    # COM + Orientation + num_dots + num_dom
                    x_cm.append(x)
                    y_cm.append(y)
                    orientation.append('V')
                    num_dots1.append(len(keypoints1))
                    num_dots2.append(len(keypoints2))
                    num_dominos = num_dominos + 1

                    # Domino half center of mass for vertical orientation:
                    xcm_h1 = x 
                    ycm_h1 = y - h//4
                    xcm_h2 = x 
                    ycm_h2 = y + h//4
                    #print('cm_h1:',xcm_h1,ycm_h1)
                    #print('cm_h2:',xcm_h2,ycm_h2)
                    xcmh.append(xcm_h1)
                    xcmh.append(xcm_h2)
                    ycmh.append(ycm_h1)
                    ycmh.append(ycm_h2)


    #cv2.imshow("Dominos Analyzed", img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    num_dots1 = np.array(num_dots1)
    num_dots2 = np.array(num_dots2)
    #num_dots = np.vstack((num_dots1,num_dots2))

    x_cm = np.array(x_cm)
    y_cm = np.array(y_cm)
    #cm = np.vstack((x_cm,y_cm))

    orientation = np.array(orientation)

    return(num_dominos, num_dots1, num_dots2, xcmh, ycmh, orientation) # Does not give actual cm, gives cm of halves, 1 after the other.
    #return(num_dominos)
#def callback():
    #num_dominos, num_dots, cm, orientation = domino_visualization()
    #return num_dominos, num_dots, cm, orientation
#num_dominos, num_dots, cm, orientation = callback()
#print('num_dominos: ', num_dominos)
#print('num_dots: ',num_dots)
#print('cm: ',cm)
#print('orientation: ',orientation)


def domino_detection():
    print("Initializing node... ")
    #rospy.init_node('domino_detection', anonymous = True)
    #rospy.Service('domino_detection', game_state, callback)
    #rospy.loginfo('Running domino_detection')
    #rospy.spin()
    
    # subscribes to the topic that publishes current position
    # Write an if statement to see when certain positions are reached
    pub_board = rospy.Publisher('/image_info', image_info, queue_size=10)
    #pub_hand = rospy.Publisher('/hand_info', game_state, queue_size = 10)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        
        num_dominos, num_dots1, num_dots2, xcmh, ycmh, orientation = domino_visualization()
        pub_string = image_info(num_dominos = num_dominos, num_dots_half1 = num_dots1, num_dots_half2 = num_dots2, x = xcmh, y = ycmh, orientation = orientation)
        pub_board.publish(pub_string)
        r.sleep()


if __name__ == '__main__':
    rospy.init_node('domino_detection', anonymous = True)
    try:
        domino_detection()
    except rospy.ROSInterruptException: pass
  

  