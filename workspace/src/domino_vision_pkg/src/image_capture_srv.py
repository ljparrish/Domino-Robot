#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from domino_vision_pkg.srv import last_image

class ImgListener:
    def __init__(self):
        self.lastImage = None

        rospy.init_node('cam_listener')

        rospy.Subscriber("/TOPIC HERE", Image, self.imgRecieved)

        rospy.Service('get_image', last_image, self.getLastImage)

    def imgRecieved(self, message):
        # Every time an image is recieved by the subscriber, we update the last image parameter
        self.lastImage = message

    def getLastImage(self, request):
        # Returns the last image seen by the camera
        print(f"Service Callback Triggered for 'get_image' at {rospy.get_time()}")
        return self.lastImage
        
if __name__ == '__main__':
    listener = ImgListener()
    print("Image Capture Service Started Successfully!")
    rospy.spin()