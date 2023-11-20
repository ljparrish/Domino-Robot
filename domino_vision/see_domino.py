import cv2
import numpy as np
import matplotlib.pyplot as plt
# TEST LINE

img = cv2.imread('domino_lots.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

ret,thresh = cv2.threshold(gray,127,255,0) # Will need to tune this value based on lighting conditions
cv2.imshow("Shapes", thresh)
cv2.waitKey(0)
cv2.destroyAllWindows()
contours,hierarchy = cv2.findContours(thresh, 1, 2)

print("Number of contours detected:", len(contours))


for cnt in contours:
   x1,y1 = cnt[0][0]
   approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True) #use 0.03 if we want to see the skinny rectangle
   if len(approx) == 4:
      x, y, w, h, theta = cv2.minAreaRect(cnt)
      ratio = float(w)/h
      # Will probably need to add a conditional statement depending on the orientation of the domino. Or use cv.minAreaRect()
      if ratio > 0.2: #0.2 is the minimum ratio to prevent seeing the middle black line 
         # Create a cropped image to count just the dots in that domino. 
         crop1 = thresh[y:y+h, x:int(np.floor(x+w/2))] 
         crop2 = thresh[y:y+h, int(np.floor(x+w/2)):int(np.floor(x+w))]

         # Dots:
         detector1 = cv2.SimpleBlobDetector_create()
         keypoints1 = detector1.detect(crop1)
         print("Black Dots Count for half 1 is:",len(keypoints1))

         detector2 = cv2.SimpleBlobDetector_create()
         keypoints2 = detector2.detect(crop2)
         print("Black Dots Count for half 2 is:",len(keypoints2))

         cv2.imshow("Cropped", crop1)
         cv2.waitKey(0)
         cv2.destroyAllWindows()

         cv2.imshow("Cropped", crop2)
         cv2.waitKey(0)
         cv2.destroyAllWindows()

         # Outline Rectangle:
         cv2.putText(img, 'Rectangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
         img = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)
         


cv2.imshow("Shapes", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
