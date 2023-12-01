#!/usr/bin/env python

from enum import Enum

class State(Enum):
    START = 1
    SETUP = 2
    LOCALIZE = 3
    FIRST_TURN = 4
    WHOS_TURN = 5
    ROBOTS_TURN = 6
    ROBOT_MOVE_TILE = 7
    ROBOT_CANT_PLAY = 8
    PLAYERS_TURN = 9
    GAME_OVER = 10


# start: let's robot know we will be playing dominos
# setup: robot spins up the necessary nodes, human sets up the robot's hand and theirs
# localize: robot moves above AR tag 13, visualizes, defines a static transform. 
# whos_turn: lets the robot know whos turn it is
# robots_turn: robot moves above hand, takes image of hand, stores data
#               robot moves above grid, takes image, saves data
#               robot iterates through grid and hand, searching for a match/valid move
# robot_move_tile: robot has a match, robot picks up domino and places it accordingly
# robot_cant_play: robot has no matches, it asks for a tile from the boneyard (from player), skips turn
# players_turn: player plays move. Robot tucks and waits for it's turn
# game over: player entered into the command line that the boneyard is empty, robot prints "thanks for playing"



