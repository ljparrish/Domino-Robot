#!/usr/bin/env python
#from gripper_ctrl.src.vac_ctrl import VacuumGripper
import robot_ctrl.src.pickandplace as pk
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion


#gripper.on()
rospy.sleep(1)
#gripper.off()
rospy.sleep(1)

# Test Poses
Pose1 = PoseStamped()
Pose1.header = Header(frame_id="base")
Pose1.pose.position = Point(0.75, 0.291, 0.03)
Pose1.pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)

Pose2 = PoseStamped()
Pose2.header = Header(frame_id="base")
Pose2.pose.position = Point(0.79, -0.114, 0.03)
Pose2.pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)

zHeight = 0.1
print("Testing Pick and Place Functions")
print("Test 1: Pick Domino")

pk.pickDomino(Pose1, zHeight)