import cv2
import numpy as np;

im = cv2.imread("domino_ideal.jpg", cv2.IMREAD_GRAYSCALE)
# create the detector with default parameters.
detector = cv2.SimpleBlobDetector_create()
 
# Detect dots.
keypoints = detector.detect(im)

print("Black Dots Count is:",len(keypoints))

# Draw detected blobs as yellow circles.
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,250,), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

cv2.imshow("Output image:", im_with_keypoints)
cv2.waitKey(0)