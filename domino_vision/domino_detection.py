import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy

def image_capture():
    cam = cv2.VideoCapture(0) 
    result, image = cam.read()
    #cv2.imshow("snapshot", image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    def domino_visualization(image):
        img = cv2.resize(image, None, fx = 0.5, fy = 0.5)
        #img = cv2.imread(image) # Get image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # Convert to grayscale
        ret,thresh = cv2.threshold(gray,160,255,0) # original: 127, 255 Apply black/white mask. Will need to tune this value based on lighting conditions
        cv2.imshow("Shapes", thresh)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        contours,hierarchy = cv2.findContours(thresh, 1, 2)
        #print("Number of contours detected:", len(contours))
        
        # Maybe add conditional statement for if no domino

        # Iterating over contours detected to determine if it is a rectangle 
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True) #use 0.03 if we want to see the skinny rectangle
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(cnt)
                ratio = float(w)/h # Rectangle's width to height ratio
                
                if w > h: # Assume horizontal orientation for domino
                    if ratio > 1.9 and ratio < 2.1: # 5 is the maximum ratio to prevent seeing the middle black line 
                        # Create a cropped image to count just the dots in that domino, splitting it in half.
                        crop1 = thresh[y:y+h, x:int(np.floor(x+w/2))] 
                        crop2 = thresh[y:y+h, int(np.floor(x+w/2)):int(np.floor(x+w))]

                        # Dots:
                        detector1 = cv2.SimpleBlobDetector_create()
                        keypoints1 = detector1.detect(crop1)
                        print("Black Dots Count for half 1/2:",len(keypoints1))

                        detector2 = cv2.SimpleBlobDetector_create()
                        keypoints2 = detector2.detect(crop2)
                        print("Black Dots Count for half 2/2:",len(keypoints2))

                        cv2.imshow("Cropped", crop1)
                        cv2.waitKey(0)
                        cv2.destroyAllWindows()

                        cv2.imshow("Cropped", crop2)
                        cv2.waitKey(0)
                        cv2.destroyAllWindows()

                        # Outline Rectangle:
                        cv2.putText(img, 'Rectangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        img = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)

                elif h > w: # assume domino is in vertical orientation 
                    if ratio > 0.4 and ratio < 0.6: #0.2 is the minimum ratio to prevent seeing the middle black line 
                        # Create a cropped image to count just the dots in that domino. 
                        crop1 = thresh[y:int(np.floor(y+h/2)), x:x+w] 
                        crop2 = thresh[int(np.floor(y+h/2)):int(np.floor(y+h)), x:x+w]

                        # Dots:
                        detector1 = cv2.SimpleBlobDetector_create()
                        keypoints1 = detector1.detect(crop1)
                        print("Black Dots Count for half 1/2:",len(keypoints1))

                        detector2 = cv2.SimpleBlobDetector_create()
                        keypoints2 = detector2.detect(crop2)
                        print("Black Dots Count for half 2/2:",len(keypoints2))

                        cv2.imshow("Cropped", crop1)
                        cv2.waitKey(0)
                        cv2.destroyAllWindows()

                        cv2.imshow("Cropped", crop2)
                        cv2.waitKey(0)
                        cv2.destroyAllWindows()

                        # Outline Rdomino_detectionectangle:
                        cv2.putText(img, 'Rectangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        img = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)
                    
                
            cv2.imshow("Dominos Analyzed", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    domino_visualization(image)
image_capture()
'''
def domino_detection():
    rospy.init_node('domino_detection')
    rospy.Service('domino_detection',,image_capture)

if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method
  print('main') 
  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('domino_detection', anonymous=True)

  rospy.Subscriber("/usb_cam/image_raw", image, image_capture) ## what are we subscribing to here?
  
  rospy.spin()
  '''

  