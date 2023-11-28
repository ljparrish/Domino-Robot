#!/usr/bin/env python
import rospy
import numpy as np
from gripper_ctrl.src.vac_ctrl import VacuumGripper
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander

class DominoRobotController():
    def __init__(self):
        rospy.wait_for_service('compute_ik')
        rospy.init_node('service_query')
        self.compute_ik = rospy.ServiceProxy('compute_ik',GetPositionIK)
        
        #Parameters
        self.zHeight = 0.1
        self.moveGroup = "right_arm"
        self.baseLink = "base"
        self.endEffectorLink = "right_hand"

    def pickDomino(self,pickPose):
        request = GetPositionIKRequest()
        request.ik_request.ik_link_name = self.endEffectorLink
        request.ik_request.pose_stamped.header.frame_id = self.baseLink
        request.ik_request.pose_stamped = pickPose
        response = self.compute_ik(request)

        group = MoveGroupCommander(self.moveGroup)

        # Move to zHeight Above Domino
        group.set_pose_target(request.ik_request.pose_stamped)
        group.shift_pose_target(2, self.zHeight)
        plan = group.plan()
        group.execute(plan[1])

        # Move down to Domino
        group.set_pose_target(request.ik_request.pose_stamped)
        self.compute_ik(request)
        plan = group.plan()
        group.execute(plan[1])

        # Pick up Domino
        #gripper.on()
        #while not gripper.isGripping():
        #    group.shift_pose_target(2, -0.005)
        #    plan = group.plan()
        #    group.execute(plan[1])

        # Retract From Table
        group.set_pose_target(request.ik_request.pose_stamped)
        group.shift_pose_target(2, self.zHeight)
        self.compute_ik(request)
        plan = group.plan()
        group.execute(plan[1])

    def placeDomino(self,placePose):
        request = GetPositionIKRequest()
        request.ik_request.ik_link_name = self.endEffectorLink
        request.ik_request.pose_stamped.header.frame_id = self.baseLink
        request.ik_request.pose_stamped = placePose
        response = self.compute_ik(request)

        group = MoveGroupCommander(self.moveGroup)

        # Move to zHeight Above Domino
        group.set_pose_target(request.ik_request.pose_stamped)
        group.shift_pose_target(2, self.zHeight)
        self.compute_ik(request)
        plan = group.plan()
        group.execute(plan[1])

        # Move down to Domino
        group.set_pose_target(request.ik_request.pose_stamped)
        self.compute_ik(request)
        plan = group.plan()
        group.execute(plan[1])

        # Drop Domino
        #gripper.off()
        rospy.sleep(2)

        # Retract From Table
        group.set_pose_target(request.ik_request.pose_stamped)
        group.shift_pose_target(2, self.zHeight)
        self.compute_ik(request)
        plan = group.plan()
        group.execute(plan[1])

# flipDomino needs tag_pos from Game_logic.py

    def flipDomino(self):
        # Predefined Domino Flipper Poses w.r.t Game Mat AR Tag
        dropPose = PoseStamped()
        dropPose.header = Header(stamp=rospy.Time.now(), frame_id=...)
        dropPose.pose.position.x = ...
        dropPose.pose.position.y = ...
        dropPose.pose.position.z = ...
        # gripper sideways left facing
        dropPose.pose.orientation.x = 0.5
        dropPose.pose.orientation.y = 0.5
        dropPose.pose.orientation.z = -0.5
        dropPose.pose.orientation.w = 0.5

        pickPose = PoseStamped()
        pickPose.header = Header(stamp=rospy.Time.now(), frame_id=...)
        pickPose.pose.position.x = ...
        pickPose.pose.position.y = ...
        pickPose.pose.position.z = ...
        # gripper vertical
        pickPose.pose.orientation.x = 0
        pickPose.pose.orientation.y = 1
        pickPose.pose.orientation.z = 0
        pickPose.pose.orientation.w = 0

        request = GetPositionIKRequest()
        request.ik_request.group_name = self.moveGroup

        request.ik_request.ik_link_name = self.endEffectorLink
        request.ik_request.pose_stamped.header.frame_id = ...

        request.ik_request.pose_stamped = dropPose

        group = MoveGroupCommander(self.moveGroup)

        group.set_pose_target(request.ik_request.pose_stamped)
        plan = group.plan()
        # Move to the Drop Pose
        group.execute(plan[1])
        #gripper.off()
        # Wait Until Pressure equalizes
        #while gripper.getCurrentPressure() < -10.0:
        #    pass

        request.ik_request.pose_stamped = pickPose
        group.set_pose_target(request.ik_request.pose_stamped)
        plan = group.plan()
        group.execute(plan[1])

        # Pick up Domino
        #gripper.on()
        #while not gripper.isGripping():
        #    group.shift_pose_target(2, -0.005)
        #    plan = group.plan()
        #    group.execute(plan[1])

        # Move Up Away from the Table
        group.shift_pose_target(2, 0.1)
        plan = group.plan()
        group.execute(plan[1])