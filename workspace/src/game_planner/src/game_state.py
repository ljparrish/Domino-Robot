#!/usr/bin/env python

from enum import Enum

class State(Enum):
    START = 1
    SETUP = 2
    WHOS_TURN = 3
    ROBOTS_TURN = 4
    ROBOT_MOVE_TILE = 5
    ROBOT_CANT_PLAY = 6
    PLAYERS_TURN = 7
    GAME_OVER = 8


# start: let's robot know we will be playing dominos
# setup: robot spins up the necessary nodes, human sets up the robot's hand and theirs
# whos_turn: lets the robot know whos turn it is
# robots_turn: robot moves above hand, takes image of hand, stores data
#               robot moves above grid, takes image, saves data
#               robot iterates through grid and hand, searching for a match/valid move
# robot_move_tile: robot has a match, robot picks up domino and places it accordingly
# robot_cant_play: robot has no matches, it asks for a tile from the boneyard (from player), skips turn
# players_turn: player plays move. Robot tucks and waits for it's turn
# game over: player entered into the command line that the boneyard is empty, robot prints "thanks for playing"



