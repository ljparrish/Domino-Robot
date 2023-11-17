import cv2

cam = cv2.VideoCapture(0) 
result, image = cam.read()
cv2.imshow("snapshot", image)
cv2.waitKey(0)
cv2.destroyAllWindows()

'''
def snapshot():
    result, image = cam.read()
    return image
    '''