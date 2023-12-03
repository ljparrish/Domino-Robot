#!/usr/bin/env python

from os import stat
import sys
import rospy
import roslaunch
from robot_ctrl.src.pickandplace import DominoRobotController
from gripper_ctrl.src.vac_ctrl import VacuumGripper
from game_planner.src.game_state import State
from domino_vision_pkg.src.image_to_world import Image_to_world

def main(args):
    rospy.init.node('game_runner', anonymous=True)
  
    state = State.START
    while not rospy.is_shutdown():
        if state == State.START:
            print("START\n")
            # enter state machine

            # define any CV objects or gamestate classes

            gripper = VacuumGripper()
            Planner = DominoRobotController(gripper)

            boardInfo = Image_to_world("/board_info")
            boardInfo.setOffset(0.016,0.014)

            handInfo = Image_to_world("/hand_info")
            handInfo.setOffset(0.016,0.014)

            # safe tuck
            Planner.safeTuck()
 
            # robot moves to start above AR tag
            x = input("Ready to set start position, please press S to execute move.\n")
            if x == "S":
                print("Moving to start position.\n")
                Planner.aboveARstartPose()
            # place board under gripper
            x = input("Please place the AR tag directly under the gripper. Reply 'D' when done.\n")
            if x == "D":
                print("Done.\n")


            state = State.SETUP
        
        elif state == State.SETUP:
            print("SETUP\n")
            print("Please clear the game board of any domino tiles\n")
            # do any set up necessary

            confCounter = 0
            # robot asks for player to give it a "hand"
            x = input("Please give me 6 domino tiles in my hand. Reply D when done.\n")
            if x == "D":
               print("Thank You.\n")
               confCounter += 1

            # verify camera angles
            print("Ready to begin camera view verification.\n")
            x = input("Enter 'C' to move the camera above the game grid.\n")
            
            if x == 'C':
                Planner.moveToBoardPicturePose()
            y = input("Is the entire 8x8 grid in the view of the camera? If not, move the grid now. If yes, reply 'Y'\n")
            
            if y == "Y":
                print("Great, ready to check the hand camera position.\n")
                confCounter += 1

            x = input("Enter 'C' to move camera above the hand.\n")
            
            if x == 'C':
                Planner.moveToHandPicturePose()
            y = input("Is the playable hand of dominoes in the view of the camera? If not, move the grid now. If yes, reply 'Y'\n")
            
            if y == 'Y':
                print("Great, let's check the starting position one last time.\n")
                confCounter += 1
                
            x = input("Enter 'C' to move the gripper.\n")
           
            if x == 'C':
                Planner.aboveARstartPose
            y = input("Is the gripper still directly over the center of the AR tag? Reply 'Y' if yes.\n")
           
            if y == 'Y':
                print("Great!\n")
                confCounter += 1

            if confCounter == 4:
            
                state = State.WHOS_TURN
  

      #  elif state == State.LOCALIZE:
       #     print("LOCALIZE\n")
        #    Planner.aboveARstartPose()
         #   state = State.WHOS_TURN

      #  elif state == State.FIRST_TURN:
        #    print("FIRST TURN\n")
            # robot always plays first

            # pick domino from hand location
            # NEED - function to get domino pose in hand
         #   Planner.pickDomino(targetPose,referenceFrame="game_board")
            # robot places it's left-most tile into the center of the grid

        #    state = State.WHOS_TURN

        
        elif state == State.WHOS_TURN:
            print("Who's turn is it?\n")
            # safe tuck, slowly
            Planner.safeTuck()
            
            x = input("Reply R for Robot's turn. Reply P for Player's turn.\n")
            # Human indicates who's turn it is
            if x == "R":
                state = State.ROBOTS_TURN
            elif x == "P":
                state = State.PLAYERS_TURN
            elif x == "Game Over":
                state = State.GAME_OVER

        elif state == State.ROBOTS_TURN:
            print("ROBOTS TURN\n")
            # move to hand, (moveTo function)
            Planner.moveToHandPicturePose()

            # image capture (image_capture.py)
            call_imageCapture()
            # hand detection (hand_detection.py)
            call_handDetection()
            # convert hand image coords to world (maybe~ image_to_world.py)
            ### TODO - Add will's Image_To_World Class
            # move to grid (moveTo)
            Planner.moveToBoardPicturePose()

            # image capture (image_capture.py)
            call_imageCapture()
            # board detection (domino_detection.py)
            call_dominoDetection()
            # convert grid coords to world (maybe~ image_to_world.py)
            ### TODO - Add will's Image_To_World Class
            # convert world coords to array indices for grid and hand (maybe in game_engine)
            # 
            # game engine runs
            # game engine will subscribe to board and hand info (world coords) (# of dots, orientation, etc.)
            # outputs a 'valid' move, position in hand, and position where to place, and orientation
            # game engine node needs to output Pose of desired place for domino

            # if no match, will return that there are no valid moves

            if match == True:
                state = State.ROBOT_MOVE_TILE
            elif match == False:
                state = State.ROBOT_CANT_PLAY 

        elif state == State.ROBOT_MOVE_TILE:
            print("ROBOT MOVE TILE\n")

            # execute move domino functions
            # move to hand, pick up, place on grid
            # when done moving tile, returns to who's turn is it, then safe tucks
            state = State.WHOS_TURN

        elif state == State.ROBOT_CANT_PLAY:
            print("ROBOT CAN'T PLAY\n")
            # robot needs a tile from the boneyard
            print("I don't have a valid domino to play. Please place a tile from the boneyard into my hand\n")
            x = input("Reply 'D' when done\n")
            if x == "D":
                # return to who's turn, safe tucks
                print("Thank you\n")
                state = State.WHOS_TURN            

        elif state == State.PLAYERS_TURN:
            print("PLAYERS TURN\n")
            state = State.WHOS_TURN

        elif state == State.GAME_OVER:
            print("GAME OVER\n")
            print("Thanks for playing with me!\n")

            # exit tasks

def call_imageCapture():
    '''
    Launches image capture node, saves an image at the current pose, then kills the node
    '''
    launch_dir = '/src/domino_vision_pkg/launch/'
    filepath = launch_dir + 'image_capture.launch'
    uuid = roslaunch.rlutil.get_or_generate_uuid(None,False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid,[filepath])
    launch.start()

def call_dominoDetection():
    '''
    Launches dominoDetection node, updates positions of dominos on the game board, then kills the node
    '''
    launch_dir = '/src/domino_vision_pkg/launch/'
    filepath = launch_dir + 'domino_detection.launch'
    uuid = roslaunch.rlutil.get_or_generate_uuid(None,False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid,[filepath])
    launch.start()

def call_handDetection():
    '''
    Launches dominoDetection node, updates positions of dominos on the game board, then kills the node
    '''
    launch_dir = '/src/domino_vision_pkg/launch/'
    filepath = launch_dir + 'hand_detection.launch'
    uuid = roslaunch.rlutil.get_or_generate_uuid(None,False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid,[filepath])
    launch.start()

if __name__ == '__main__':
    main(sys.argv)