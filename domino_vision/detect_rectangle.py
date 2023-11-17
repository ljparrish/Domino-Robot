import cv2
import numpy as np

img = cv2.imread('domino_ideal_horiz.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

ret,thresh = cv2.threshold(gray,127,255,0) # Will need to tune this value based on lighting conditions

cv2.imshow("Shapes", thresh)
cv2.waitKey(0)
cv2.destroyAllWindows()

contours,hierarchy = cv2.findContours(thresh, 1, 2)

print("Number of contours detected:", len(contours))

# Need someway to detect the larger rectangles from the contours. Only keep the larger ones. 

for cnt in contours:
   x1,y1 = cnt[0][0]
   approx = cv2.approxPolyDP(cnt, 0.1*cv2.arcLength(cnt, True), True) #use 0.03 if we want to see the skinny rectangle
   if len(approx) == 4:
      x, y, w, h = cv2.boundingRect(cnt)
      ratio = float(w)/h
      print(ratio)
      if ratio >= 0.9 and ratio <= 1.1:
         img = cv2.drawContours(img, [cnt], -1, (0,255,255), 3)
         cv2.putText(img, 'Circle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
      elif ratio > 4/34:
         #do something with dots

         cv2.putText(img, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
         img = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)
      '''
      else:
         cv2.putText(img, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
         img = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)
         '''

cv2.imshow("Shapes", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
