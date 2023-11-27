#!/usr/bin/env python
import rospy
import numpy as np
import time
from gripper_ctrl.src.vac_ctrl import VacuumGripper
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander

def initalizeIK():
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')

    compute_ik = rospy.ServiceProxy('compute_ik',GetPositionIK)
    group = MoveGroupCommander("right_arm")
    gripper = VacuumGripper()
    return compute_ik, group, gripper

def pickDomino(pickPose, group, zHeight, gripper):
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"

    link = "right_hand"

    request.ik_request.ik_link_name = link
    request.ik_request.pose_stamped.header.frame_id = ...

    # Move to zHeight Above Domino
    request.ik_request.pose_stamped = pickPose
    group.set_pose_target(pickPose)
    group.shift_pose_target([0, 0, 1],zHeight)
    plan = group.plan()
    group.execute(plan[1])

    # Move down to Domino
    group.set_pose_target(pickPose)
    plan = group.plan()
    group.execute(plan[1])

    # Pick up Domino
    gripper.on()
    while not gripper.isGripping():
        group.shift_pose_target([0, 0, -1],0.005)
        plan = group.plan()
        group.execute(plan[1])

    # Retract From Table
    group.set_pose_target(pickPose)
    group.shift_pose_target([0, 0, 1],zHeight)
    plan = group.plan()
    group.execute(plan[1])

def placeDomino(placePose, group, zHeight, gripper):
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"

    link = "right_hand"

    request.ik_request.ik_link_name = link
    request.ik_request.pose_stamped.header.frame_id = ...

    # Move to zHeight Above Domino
    request.ik_request.pose_stamped = placePose
    group.set_pose_target(placePose)
    group.shift_pose_target([0, 0, 1],zHeight)
    plan = group.plan()
    group.execute(plan[1])

    # Move down to Domino
    group.set_pose_target(placePose)
    plan = group.plan()
    group.execute(plan[1])

    # Drop Domino
    gripper.off()
    time.sleep(2)

    # Retract From Table
    group.set_pose_target(placePose)
    group.shift_pose_target([0, 0, 1],zHeight)
    plan = group.plan()
    group.execute(plan[1])

def flipDomino(group, gripper):
    # Predefined Domino Flipper Poses w.r.t Game Mat AR Tag
    dropPose = PoseStamped()
    dropPose.header = Header(stamp=rospy.Time.now(), frame_id=...)
    dropPose.pose.position.x = ...
    dropPose.pose.position.y = ...
    dropPose.pose.position.z = ...
    dropPose.pose.orientation.x = ...
    dropPose.pose.orientation.y = ...
    dropPose.pose.orientation.z = ...
    dropPose.pose.orientation.w = ...

    pickPose = PoseStamped()
    pickPose.header = Header(stamp=rospy.Time.now(), frame_id=...)
    pickPose.pose.position.x = ...
    pickPose.pose.position.y = ...
    pickPose.pose.position.z = ...
    pickPose.pose.orientation.x = ...
    pickPose.pose.orientation.y = ...
    pickPose.pose.orientation.z = ...
    pickPose.pose.orientation.w = ...

    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"

    link = "right_hand"

    request.ik_request.ik_link_name = link
    request.ik_request.pose_stamped.header.frame_id = ...

    request.ik_request.pose_stamped = dropPose
    group.set_pose_target(dropPose)
    plan = group.plan()
    # Move to the Drop Pose
    group.execute(plan[1])
    gripper.off()
    # Wait Until Pressure equalizes
    while gripper.getCurrentPressure() < -10.0:
        pass

    request.ik_request.pose_stamped = pickPose
    group.set_pose_target(pickPose)
    plan = group.plan()
    group.execute(plan[1])

    # Pick up Domino
    gripper.on()
    while not gripper.isGripping():
        group.shift_pose_target([0, 0, -1],0.005)
        plan = group.plan()
        group.execute(plan[1])

    # Move Up Away from the Table
    group.shift_pose_target([0, 0, 1],0.1)
    plan = group.plan()
    group.execute(plan[1])
    
