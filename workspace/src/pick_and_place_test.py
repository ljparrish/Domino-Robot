#!/usr/bin/env python
#from gripper_ctrl.src.vac_ctrl import VacuumGripper
from robot_ctrl.src.pickandplace import DominoRobotController
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion

rospy.init_node('service_query')
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


Planner = DominoRobotController()

#print("Test 1: moveTo")
#Planner.moveTo(Pose1)
#Planner.moveTo(Pose2)

#print("Test 2:pickDomino")
#Planner.pickDomino(Pose1)

#print("Test 3:placeDomino")
#Planner.placeDomino(Pose2)

print("Test 4:getARPose")
AR_Pose = Planner.getARPose()
print(AR_Pose)