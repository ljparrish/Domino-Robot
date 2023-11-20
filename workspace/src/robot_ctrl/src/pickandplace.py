#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse

def main(pickLocation, placeLocation):
    # Input definitions:
    # - pickLocation: tf of pick up location relative to baseframe
    # - placeLocation: tf of place location relative to baseframe
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')

    compute_ik = rospy.ServiceProxy('compute_ik',GetPositionIK)

    zMoveHeight = 0.5 # Set height above table here during pick and place operations
    # Set Pose Arrays
    # Positions
    x = np.array([pickLocation.pose.position.x, pickLocation.pose.position.x, placeLocation.pose.position.x, placeLocation.pose.position.x])
    y = np.array([pickLocation.pose.position.y, pickLocation.pose.position.y, placeLocation.pose.position.y, placeLocation.pose.position.y])
    z = np.array([pickLocation.pose.position.z, pickLocation.pose.position.z + zMoveHeight, placeLocation.pose.position.z + zMoveHeight, placeLocation.pose.position.z])

    # Orientations
    qx = np.array([pickLocation.pose.orientation.x, pickLocation.pose.orientation.x, placeLocation.pose.orientation.x, placeLocation.pose.orientation.x])
    qy = np.array([pickLocation.pose.orientation.y, pickLocation.pose.orientation.y, placeLocation.pose.orientation.y, placeLocation.pose.orientation.y])
    qz = np.array([pickLocation.pose.orientation.z, pickLocation.pose.orientation.z, placeLocation.pose.orientation.z, placeLocation.pose.orientation.z])
    qw = np.array([pickLocation.pose.orientation.w, pickLocation.pose.orientation.w, placeLocation.pose.orientation.w, placeLocation.pose.orientation.w])

    while not rospy.is_shutdown():
        for i in range(len(x)):
            request = GetPositionIKRequest()
            request.ik_request.group_name = "right_arm"

            link = "right_gripper_tip"

            request.ik_request.ik_link_name = link
            request.ik_request.pose_stamped.header.frame_id = "base"

            request.ik_request.pose_stamped.pose.position.x = x[i]
            request.ik_request.pose_stamped.pose.position.y = y[i]
            request.ik_request.pose_stamped.pose.position.z = z[i]
            request.ik_request.pose_stamped.pose.orientation.x = qx[i]
            request.ik_request.pose_stamped.pose.orientation.y = qy[i]
            request.ik_request.pose_stamped.pose.orientation.z = qz[i]
            request.ik_request.pose_stamped.pose.orientation.w = qw[i]


            




if __name__ == '__main__':
    main()