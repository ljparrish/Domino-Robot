#!/usr/bin/env python
import rospy
import numpy as np
from gripper_ctrl.src.vac_ctrl import VacuumGripper
import tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander
from sensor_msgs.msg import JointState

class DominoRobotController():
    def __init__(self, Gripper):
        rospy.wait_for_service('compute_ik')
        #rospy.init_node('service_query')
        self.compute_ik = rospy.ServiceProxy('compute_ik',GetPositionIK)

        #Parameters
        self.zHeight = 0.1
        self.moveGroup = "right_arm"
        self.baseLink = "base"
        self.endEffectorLink = "right_hand"
        assert type(Gripper == VacuumGripper)
        self.gripper = Gripper

    def set_ZHeight(self,Z):
        self.zHeight = Z

    def moveTo(self, StampedPose, debug=True, referenceFrame="base", targetFrame="right_hand"):
        request = GetPositionIKRequest()
        request.ik_request.ik_link_name = targetFrame
        request.ik_request.pose_stamped.header.frame_id = referenceFrame
        request.ik_request.pose_stamped = StampedPose
        try:
            response = self.compute_ik(request)
            if debug:
                print("-----IK Response-----")
                print(response)

            group = MoveGroupCommander(self.moveGroup)
            group.set_end_effector_link(targetFrame) # NEW LINE HERE We change the end effector link
            group.set_pose_target(request.ik_request.pose_stamped)
            plan = group.plan()
            if debug:
                if not input("Does the Path Look Safe? [Y/N]") == "Y":
                   print("Restart Node")
                   rospy.spin()
            group.execute(plan[1])
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def moveToJoint(self, joint, debug=True):
        try:
            group = MoveGroupCommander(self.moveGroup)
            group.set_joint_value_target(joint)
            group.set_start_state_to_current_state()

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
        self.gripper.on()
        while not self.gripper.isGripping():
            pickPose.pose.position.z -= 0.005
            self.moveTo(pickPose, debug=False)
            rospy.sleep(0.1)
        rospy.sleep(2)

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
        self.gripper.off()
        rospy.sleep(2)

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
        cameraJointState = JointState()
        cameraJointState.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6']
        cameraJointState.position = [1.22222, 0.87835, -1.48364, 1.73790, 2.36236, -0.01118, 3.34471]
        
        self.moveToJoint(cameraJointState)
        #return AR_Pose
       
        # get AR_Pose by looking up transform between AR tag & wrist?
        # may have to move the tf buffer initialization to the "set up" node
        tfBuffer = tf2_ros.Buffer()    
        tfListener = tf2_ros.TransformListener(tfBuffer)  

        try:
            # lookup the transform and save it in trans
            trans = tfBuffer.lookup_transform('base', 'ar_marker_13' ,rospy.Time(0), rospy.Duration(10))
            trans.header.frame_id = "base"
            trans.child_frame_id = "game_board"
        except Exception as e:
            print(e)
            print("Retrying ...")

        AR_Pose = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
        game_board = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
        # np.array(AR_Pose)
    
        # make 'game_board' frame that is where AR tag is, publish static transform

        #tfBuffer = tf2_ros.Buffer()    
        #tfListener = tf2_ros.TransformListener(tfBuffer)

        trans.transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)

        try:
            # lookup the transform and save it in trans
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            broadcaster.sendTransform(trans)

            print('not a failure :)') 
        except Exception as e:
            print(e)
            print("Retrying ...")


    def moveToHandPicturePose(self):
        cameraJointState = JointState()
        cameraJointState.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6']
        cameraJointState.position = [0.52847, -0.28549, -0.17221, 1.23390, 0.47744, -0.97401, 3.17143]
        
        self.moveToJoint(cameraJointState)

    def moveToBoardPicturePose(self):
        cameraJointState = JointState()
        cameraJointState.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6']
        cameraJointState.position = [-0.03083, -0.59538, 0.04406, 1.55704, -0.07918, -1.023679, 3.30380]
        
        self.moveToJoint(cameraJointState)
