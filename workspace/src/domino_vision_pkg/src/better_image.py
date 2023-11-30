#! /usr/bin/env python
import argparse
import numpy as np
import os 
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from intera_interface import Cameras

cameras = Cameras('right_hand_camera')

