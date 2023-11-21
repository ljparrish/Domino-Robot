import cv2
import numpy as np
import matplotlib.pyplot as plt

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

            # Outline Rectangle:
            cv2.putText(img, 'Rectangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            img = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)
         
         
cv2.imshow("Dominos Analyzed", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
