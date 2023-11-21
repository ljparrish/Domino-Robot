import rospy
from domino_vision_pkg.srv import game_state #service type

def game_planner():
    rospy.init_node('game_planner_client')
    rospy.wait_for_service('domino_detection')
    try:
        proxy = rospy.ServiceProxy('domino_detection',game_state)
        proxy()
    except rospy.ServiceException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    game_planner()