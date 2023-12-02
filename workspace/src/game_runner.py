#!/usr/bin/env python

from os import stat
import sys
import rospy
from robot_ctrl.src.pickandplace import DominoRobotController
from gripper_ctrl.src.vac_ctrl import VacuumGripper
from game_planner.src.game_state import State
from domino_vision_pkg.src.image_capture import ImageCapture
from domino_vision_pkg.src.hand_detection import HandDetection

def main(args):
    rospy.init.node('game_runner', anonymous=True)
    # launch file

    state = State.START
    while not rospy.is_shutdown():
        if state == State.START:
            print("START\n")
            # enter state machine

            gripper = VacuumGripper()
            Planner = DominoRobotController(gripper)

            # safe tuck
            Planner.safeTuck()
            
            # define any CV objects or gamestate classes
            state = State.SETUP
        
        elif state == State.SETUP:
            print("SETUP\n")
            print("Please clear the game board of any domino tiles\n")
            # do any set up necessary

            # robot asks for player to give it a "hand"
            x = input("Please give me 6 domino tiles in my hand. Reply D when done.\n")
            if x == "D":
               print("Thank You.\n")

            # robot moves to start above AR tag
            x = input("Ready to set start position, please press S to execute move.\n")
            if x == "S":
                print("Moving to start position.\n")
                Planner.aboveARstartPose()
            
            # place board under gripper
            x = input("Please place the AR tag directly under the gripper. Reply 'D' when done.\n")
            if x == "D":
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
            # hand detection (hand_detection.py)
            # convert hand image coords to world (maybe~ image_to_world.py)
            # move to grid (moveTo)
            Planner.moveToBoardPicturePose()

            # image capture (image_capture.py)
            # board detection (domino_detection.py)
            # convert grid coords to world (maybe~ image_to_world.py)
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



if __name__ == '__main__':
    main(sys.argv)