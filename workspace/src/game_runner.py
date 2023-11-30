#!/usr/bin/env python

from os import stat
import sys
import rospy
# from jenga_bot import Jenga_Bot
from game_state import State

def main(args):
    rospy.init.node('game_runner', anonymous=True)
    # jenga_bot = Jenga_Bot()

    state = State.START
    while not rospy.is_shutdown():
        if state == State.START:
            print("START\n")
            # enter state machine
        
        elif state == State.SETUP:
            print("SETUP\n")
            # do any set up necessary
            pass

        elif state == State.LOCALIZE:
            print("LOCALIZE\n")
            # move to wrist position where camera can visualize AR tag
            # get transform from base to AR tag, publish static transform to base game board off of
            pass
        
        elif state == State.WHOS_TURN:
            print("Who's turn is it?\n")
            x = input("Reply R for Robot's turn. Reply P for Player's turn.\n")
            # need command line arg syntax
            if x == "R":
                state = State.ROBOTS_TURN
            elif x == "P":
                state = State.PLAYERS_TURN
            elif x == "Game Over":
                state = State.GAME_OVER

        elif state == State.ROBOTS_TURN:
            print("ROBOTS TURN\n")
            # move to habd, get info, move to grid, get info
            # iterate through valid moves, search through match

            # if theres a match
            if match == True:
                state = State.ROBOT_MOVE_TILE
            elif match == False:
                state = State.ROBOT_CANT_PLAY 

        elif state == State.ROBOT_MOVE_TILE:
            print("ROBOT MOVE TILE\n")

            # execute move domino functions
            # when done moving tile, returns to who's turn is it
            state = State.WHOS_TURN

        elif state == State.ROBOT_CANT_PLAY:
            print("ROBOT CAN'T PLAY\n")
            # robot needs a tile from the boneyard
            print("Please place a tile from the boneyard into my hand\n")
            x = input("Reply 'D' when done\n")
            if x == "D":
                # return to who's turn
                print("Thank you\n")
                state = State.WHOS_TURN            

        elif state == State.PLAYERS_TURN:
            print("PLAYERS TURN\n")
            # robot tucks? then asks who's turn it is again

            state = State.WHOS_TURN

        elif state == State.GAME_OVER:
            print("GAME OVER\n")
            print("Thanks for playing with me!\n")
            # robot tucks?

            # exit tasks



if __name__ == '__main__':
    main(sys.argv)