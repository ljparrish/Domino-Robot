#!/usr/bin/env python
import rospy
import numpy as np
from gripper_ctrl.src.vac_ctrl import VacuumGripper
import tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander

class DominoRobotController():
    def __init__(self):
        rospy.wait_for_service('compute_ik')
        #rospy.init_node('service_query')
        self.compute_ik = rospy.ServiceProxy('compute_ik',GetPositionIK)
        
        #Parameters
        self.zHeight = 0.3
        self.moveGroup = "right_arm"
        self.baseLink = "base"
        self.endEffectorLink = "right_hand"

    def moveTo(self, StampedPose, debug=True, referenceFrame="base"):
        request = GetPositionIKRequest()
        request.ik_request.ik_link_name = self.endEffectorLink
        request.ik_request.pose_stamped.header.frame_id = referenceFrame
        request.ik_request.pose_stamped = StampedPose
        try:
            response = self.compute_ik(request)
            if debug:
                print("-----IK Response-----")
                print(response)

            group = MoveGroupCommander(self.moveGroup)
            group.set_pose_target(request.ik_request.pose_stamped)
            plan = group.plan()
            if debug:
                if not input("Does the Path Look Safe? [Y/N]") == "Y":
                   print("Restart Node")
                   rospy.spin()
            group.execute(plan[1])
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def pickDomino(self,pickPose):
        
        # Move to zHeight Above Domino
        pickPose.pose.position.z += self.zHeight
        self.moveTo(pickPose, debug=False)
        
        # Move down to Domino
        pickPose.pose.position.z -= self.zHeight
        self.moveTo(pickPose, debug=False)

        # Pick up Domino
        #gripper.on()
        #while not gripper.isGripping():
        #    group.shift_pose_target(2, -0.005)
        #    plan = group.plan()
        #    group.execute(plan[1])

        # Retract From Table
        pickPose.pose.position.z += self.zHeight
        self.moveTo(pickPose, debug=False)

    def placeDomino(self,placePose):

        # Move to zHeight Above Place Pose
        placePose.pose.position.z += self.zHeight
        self.moveTo(placePose, debug=False)

        # Move down to Domino
        placePose.pose.position.z -= self.zHeight
        self.moveTo(placePose, debug=False)
        
        # Drop Domino
        #gripper.off()
        #rospy.sleep(2)

        # Retract From Table
        placePose.pose.position.z += self.zHeight
        self.moveTo(placePose, debug=False)
        

    # flipDomino needs tag_pos from Game_logic.py

    def flipDomino(self):
        # Predefined Domino Flipper Poses w.r.t Game Mat AR Tag
        dropPose = PoseStamped()
        dropPose.header = Header(stamp=rospy.Time.now(), frame_id=...)
        dropPose.pose.position.x = 0.510
        dropPose.pose.position.y = 0.069
        dropPose.pose.position.z = 0.097
        # gripper sideways left facing
        dropPose.pose.orientation.x = 0.5
        dropPose.pose.orientation.y = 0.5
        dropPose.pose.orientation.z = -0.5
        dropPose.pose.orientation.w = 0.5

        pickPose = PoseStamped()
        pickPose.header = Header(stamp=rospy.Time.now(), frame_id=...)
        pickPose.pose.position.x = 0.665
        pickPose.pose.position.y = 0.077
        pickPose.pose.position.z = 0.036
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

    def getARPose(self):
        AR_Pose = PoseStamped()
        cameraPose = PoseStamped()
        cameraPose.header.frame_id = self.baseLink
        cameraPose.pose.position = Point(0.500, 0.426, 0.150)
        cameraPose.pose.orientation = Quaternion(0.748, -0.660, -0.026, -0.019)
        
        self.moveTo(cameraPose)
        #return AR_Pose
       
        # get AR_Pose by looking up transform between AR tag & wrist?
        # copied over from game_engine_node.py
        tfBuffer = tf2_ros.Buffer()    
        tfListener = tf2_ros.TransformListener(tfBuffer)  

        try:
            # lookup the transform and save it in trans
            trans = tfBuffer.lookup_transform('base', 'ar_marker_13' ,rospy.Time(0), rospy.Duration(10)) 
        except Exception as e:
            print(e)
            print("Retrying ...")

        AR_Pose = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
        return np.array(AR_Pose)

        # now need to publish a static transform
        # this is probably wrong, from lab 6
        tf2_ros.static_transform_publisher('base', 'ar_marker_13')


    def moveToHandPicturePosition(self):
        #pass
        handPose = PoseStamped()
        handPose.header = Header(stamp=rospy.Time.now(), frame_id=...)
        handPose.pose.position.x = 0.013
        handPose.pose.position.y = 0.208
        handPose.pose.position.z = 0.373
        # gripper downward facing
        handPose.pose.orientation.x = 1
        handPose.pose.orientation.y = 0
        handPose.pose.orientation.z = 0
        handPose.pose.orientation.w = 0

        # still need to get it to move there?

    def moveToUpperBoardPicturePosition(self):
        #pass
        upper_board_Pose = PoseStamped()
        upper_board_Pose.header = Header(stamp=rospy.Time.now(), frame_id=...)
        upper_board_Pose.pose.position.x = 0.344
        upper_board_Pose.pose.position.y = 0.344
        upper_board_Pose.pose.position.z = 0.366
        # gripper downward facing
        upper_board_Pose.pose.orientation.x = 1
        upper_board_Pose.pose.orientation.y = 0
        upper_board_Pose.pose.orientation.z = 0
        upper_board_Pose.pose.orientation.w = 0

        # still need to get it to move there?

    def moveToLowerBoardPicturePosition(self):
        #pass

        lower_board_Pose = PoseStamped()
        lower_board_Pose.header = Header(stamp=rospy.Time.now(), frame_id=...)
        lower_board_Pose.pose.position.x = 0.282
        lower_board_Pose.pose.position.y = 0.156
        lower_board_Pose.pose.position.z = 0.408
        # gripper downward facing
        lower_board_Pose.pose.orientation.x = 1
        lower_board_Pose.pose.orientation.y = 0
        lower_board_Pose.pose.orientation.z = 0
        lower_board_Pose.pose.orientation.w = 0

        # still need to get it to move there?