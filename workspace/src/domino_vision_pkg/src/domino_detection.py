import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy

#from domino_vision_pkg.srv import game_state #service type

def image_capture():
    cam = cv2.VideoCapture(0) 
    s, im = cam.read()
    #cv2.imshow("Test", im)
    cv2.imwrite("test.jpg", im)

    #image = im      
    #with open("rect_test.py") as f:
        #exec(f.read())
    
    #result, image = cam.read()
    #cv2.imshow("snapshot", image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

def rect_test():
    img = cv2.imread('test.jpg') # Get image
    img = cv2.resize(img, None, fx = 0.5, fy = 0.5)
    img = cv2.GaussianBlur(img,(5,5),0)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # Convert to grayscale
    ret,thresh = cv2.threshold(gray,220,255,0) # Apply black/white mask. Will need to tune this value based on lighting conditions
    cv2.imshow("Shapes", thresh)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    contours,hierarchy = cv2.findContours(thresh, 1, 2)
    #print("Number of contours detected:", len(contours))

    # Iterating over contours detected to determine if it is a rectangle 
    num_dominos = 0
    num_dots1 = []
    num_dots2 = []
    x_cm = []
    y_cm = []
    orientation = []
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True) #use 0.03 if we want to see the skinny rectangle
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)
            print(x)
            ratio = float(w)/h # Rectangle's width to height ratio
            x_cm.append(x+(w/2))
            y_cm.append(y+(h/2))
            if w > h: # Assume horizontal orientation for domino
                orientation.append('H')
                if ratio > 1.9 and ratio < 2.1: # 5 is the maximum ratio to prevent seeing the middle black line 
                    # Create a cropped image to count just the dots in that domino, splitting it in half.
                    crop1 = thresh[y:y+h, x:int(np.floor(x+w/2))] 
                    crop2 = thresh[y:y+h, int(np.floor(x+w/2)):int(np.floor(x+w))]

                    # Dots:
                    detector1 = cv2.SimpleBlobDetector_create()
                    keypoints1 = detector1.detect(crop1)
                    print("Black Dots Count for half 1/2:",len(keypoints1))
                    num_dots1.append(len(keypoints1))
                    

                    detector2 = cv2.SimpleBlobDetector_create()
                    keypoints2 = detector2.detect(crop2)
                    print("Black Dots Count for half 2/2:",len(keypoints2))
                    num_dots2.append(len(keypoints2))
                    
                    cv2.imshow("Cropped", crop1)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    cv2.imshow("Cropped", crop2)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    # Outline Rectangle:
                    cv2.putText(img, 'Rectangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    img = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)
                    num_dominos = num_dominos + 1

            elif h > w: # assume domino is in vertical orientation 
                orientation.append('V')
                if ratio > 0.4 and ratio < 0.6: #0.2 is the minimum ratio to prevent seeing the middle black line 
                    # Create a cropped image to count just the dots in that domino. 
                    crop1 = thresh[y:int(np.floor(y+h/2)), x:x+w] 
                    crop2 = thresh[int(np.floor(y+h/2)):int(np.floor(y+h)), x:x+w]

                    # Dots:
                    detector1 = cv2.SimpleBlobDetector_create()
                    keypoints1 = detector1.detect(crop1)
                    print("Black Dots Count for half 1/2:",len(keypoints1))
                    num_dots1.append(len(keypoints1))

                    detector2 = cv2.SimpleBlobDetector_create()
                    keypoints2 = detector2.detect(crop2)
                    print("Black Dots Count for half 2/2:",len(keypoints2))
                    num_dots2.append(len(keypoints2))

                    cv2.imshow("Cropped", crop1)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    cv2.imshow("Cropped", crop2)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    # Outline Rectangle:
                    cv2.putText(img, 'Rectangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    img = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)
                    num_dominos = num_dominos + 1   

    cv2.imshow("Dominos Analyzed", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    num_dots1 = np.array(num_dots1)
    num_dots2 = np.array(num_dots2)
    num_dots = np.vstack((num_dots1,num_dots2))

    x_cm = np.array(x_cm)
    y_cm = np.array(y_cm)
    cm = np.vstack((x_cm,y_cm))

    orientation = np.array(orientation)

    return(num_dominos, num_dots, cm, orientation)
def callback():
    image_capture()
    num_dominos, num_dots, cm, orientation = rect_test()
    return num_dominos, num_dots, cm, orientation
num_dominos, num_dots, cm, orientation = callback()
print(num_dominos)
print(num_dots)
print(cm)
print(orientation)

'''
def domino_detection():
    rospy.init_node('domino_detection')
    rospy.Service('domino_detection', game_state, image_capture)
    rospy.loginfo('Running domino_detection')
    rospy.spin()

if __name__ == '__main__':

  domino_detection()
  
'''
  