#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from domino_vision_pkg.srv import last_image, image_info, position_state



def initializeServices():
    print("Setting up Image Capture SRV, waiting for service node ... ")
    rospy.wait_for_service('get_image')
    rospy.sleep(1)
    print("Successfully setup Image Capture SRV!")
    image_capture_srv = rospy.ServiceProxy('get_image', last_image)

    print("Setting up Detect Dominos SRV, waiting for service node ... ")
    rospy.wait_for_service('detect_dominos')
    rospy.sleep(1)
    print("Successfully setup Detect Dominos SRV!")
    domino_detection_srv = rospy.ServiceProxy('detect_dominos', image_info)

    print("Setting up Image To Board SRV, waiting for service node ... ")
    rospy.wait_for_service('image_to_board')
    rospy.sleep(1)
    print("Successfully setup Image To Board SRV!")
    board_srv = rospy.ServiceProxy('image_to_board', position_state)

    print("Setting up Image To Hand SRV, waiting for service node ... ")
    rospy.wait_for_service('image_to_hand')
    rospy.sleep(1)
    print("Successfully setup Image To Hand SRV!")
    hand_srv = rospy.ServiceProxy('image_to_hand', position_state)

    print("All services online, ready for testing!")
    return image_capture_srv, domino_detection_srv, board_srv, hand_srv

def main():
    image_capture_srv, domino_detection_srv, board_srv, hand_srv = initializeServices()
    while not rospy.is_shutdown():
        print("Test 1: Image Capture")
        input("Position the Sawyer Arm over the game board area, then enter any key to continue")
        Image = image_capture_srv().image_data
        print(Image)

        print("Test 2: Domino Detection on Board")
        input("Enter any key to analyze board ... ")
        board_dominos = domino_detection_srv(Image)
        print(board_dominos)

        print("Test 3: Domino Positions on Board")
        input("Enter any key to compute positions ... ")
        board_positions = board_srv(board_dominos)
        print(board_positions)

        print("Test 4: Domino Hand Perception")
        input("Move the camera above the robots hand, then enter any key")
        Image = image_capture_srv().image_data
        hand_dominos = domino_detection_srv(Image)
        hand_positions = hand_srv(hand_dominos)
        print("Image Data:\n")
        print(Image)
        print("Hand Domino Data\n")
        print(hand_dominos)
        print("Hand Position Data\n")
        print(hand_positions)

if __name__ == '__main__':
    rospy.init_node('perception_test', Anonymous = True)
    main()