#!/usr/bin/env python

import numpy as np
import random
import rospy
from domino_vision_pkg.msg import game_state #msg type
# import ar_track_alvar
import tf2_ros

class GameEngine:
    def __init__(self):
        rospy.init_node('game_engine', anonymous = True)
        
        self.num_hand_dominos = None
        self.hand_dots_half1 = None
        self.hand_dots_half2 = None
        self.hand_dom_x_cm = None
        self.hand_dom_y_cm = None
        self.hand_dom_orientation = None

        self.num_board_dominos = None
        self.board_dots_half1 = None
        self.board_dots_half2 = None
        self.board_dom_x_cm = None
        self.board_dom_y_cm = None
        self.board_dom_orientation = None 
        

        rospy.Subscriber("/image_info",game_state, self.board_converter)
        rospy.Subscriber("/hand_info",game_state, self.hand_converter)
            

        rospy.spin()
    ## Create an empty grid that will be filled in with domino values
    def initialize_board(self):
        # Initialize a 2D array for the domino board
        return [[' ' for _ in range(10)] for _ in range(10)]

    def print_board(self,board):
        # Print the current state of the domino board
        for row in board:
            print('|' + '|'.join(str(cell) if cell != ' ' else ' ' for cell in row) + '|')
            print('-' * 17)

    # Function places domino at specified position in the array
    def place_domino(self, board, domino, row, col, orientation):
        # Place the domino at the specified position on the board
        if orientation == 'h':
            board[row][col] = domino[0]
            board[row][col + 1] = domino[1]
        elif orientation == 'v':
            board[row][col] = domino[0]
            board[row + 1][col] = domino[1]


    def valid_move(self, board, hand_domino, board_domino, position, orientation):
        """
        Check if placing a domino on the board at the specified position is feasible.

        Parameters:
        - board: The current state of the domino board.
        - hand_domino: The domino to be placed.
        - board_domino: The adjacent domino already on the board
        - position: position of the adjacent domino
        - orientation: orientation of the adjacent domino

        Returns:
        - feasibility: True if the placement is feasible, False otherwise.
        - played_orientation: The preferred orientation ('h' for horizontal, 'v' for vertical).
        - played_position: The preferred position of where to place the hand_domino
        """
        rows, cols = np.shape(board)
        top_half=hand_domino[0]
        bottom_half = hand_domino[1]
        played_position = None
        match_found = False

        # Check feasibility for both orientations
        if top_half == board_domino[0]:
            if orientation == 'v':
                # plays feasible option 1
                if board[position[0]][position[1]+1]  == ' ' and board[position[0]][position[1]+2]  == ' 'and \
                board[position[0]][position[1]+3]  == ' 'and board[position[0]+1][position[1]+1]  == ' 'and \
                board[position[0]+1][position[1]+2]  == ' ' and board[position[0]-1][position[1]+1] == ' 'and \
                board[position[0]-1][position[1]+2] == '':
                    played_orientation = "h"
                    played_position = np.array([[position[0],position[1]+1],[position[0],position[1]+2]])
                    match_found = True
                # plays feasible option 3
                elif board[position[0]-1][position[1]]  == ' ' and board[position[0]-1][position[1]+1]  == ' 'and \
                board[position[0]-1][position[1]+2]  == ' 'and board[position[0]+1][position[1]+1]  == ' 'and \
                board[position[0]-2][position[1]]  == ' ' and board[position[0]-2][position[1]+1] == ' 'and \
                board[position[0]-1][position[1]-1] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0]-1,position[1]],[position[0]-1,position[1]+1]]) 
                    match_found = True
                # plays feasible option 4
                elif board[position[0]-1][position[1]-1]  == ' ' and board[position[0]-1][position[1]-2]  == ' 'and \
                    board[position[0]-1][position[1]-3]  == ' 'and board[position[0]-1][position[1]+1]  == ' 'and \
                    board[position[0]-2][position[1]+1]  == ' ' and board[position[0]-1][position[1]-1] == ' 'and \
                    board[position[0]-2][position[1]-1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]-1,position[1]],[position[0]-2,position[1]]])
                    match_found = True
                # plays feasible option 5
                elif board[position[0]-1][position[1]]  == ' ' and board[position[0]-1][position[1]-1]  == ' 'and \
                    board[position[0]-1][position[1]+1]  == ' 'and board[position[0]-1][position[1]-2]  == ' 'and \
                    board[position[0]][position[1]-1]  == ' ' and board[position[0]-2][position[1]] == ' 'and \
                    board[position[0]-2][position[1]-1] == '':
                    played_orientation = "h"
                    played_position = np.array([[position[0]-1,position[1]],[position[0]-1,position[1]-1]])
                    match_found = True
                # plays feasible option 7
                elif board[position[0]][position[1]-1]  == ' ' and board[position[0]][position[1]-2]  == ' 'and \
                    board[position[0]][position[1]-3]  == ' 'and board[position[0]-1][position[1]-1]  == ' 'and \
                    board[position[0]-1][position[1]-2]  == ' ' and board[position[0]+1][position[1]-1] == ' 'and \
                    board[position[0]+1][position[1]-2] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0],position[1]-1],[position[0],position[1]-2]])
                    match_found = True
            elif orientation == 'h':
                # plays feasible option 1
                if board[position[0]-1][position[1]] == ' ' and board[position[0]-2][position[1]] == ' ' and \
                board[position[0]-3][position[1]] == ' ' and board[position[0]-1][position[1]-1] == ' ' and \
                board[position[0]-2][position[1]-1]== ' ' and board[position[0]-1][position[1]+1] == ' 'and \
                board[position[0]-2][position[1]+1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]-1,position[1]],[position[0]-2,position[1]]])
                    match_found = True
                # plays feasible option 3
                elif board[position[0]][position[1]-1]  == ' ' and board[position[0]-1][position[1]-1]  == ' 'and \
                board[position[0]-1][position[1]-2]  == ' 'and board[position[0]+1][position[1]-1]  == ' 'and \
                board[position[0]-1][position[1]]  == ' ' and board[position[0]][position[1]-2] == ' 'and \
                board[position[0]-1][position[1]-2] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0],position[1]-1],[position[0]-1,position[1]-1]]) 
                    match_found = True
                # plays feasible option 4
                elif board[position[0]][position[1]-1]  == ' ' and board[position[0]][position[1]-2]  == ' 'and \
                    board[position[0]][position[1]-3]  == ' 'and board[position[0]-1][position[1]-1]  == ' 'and \
                    board[position[0]-1][position[1]-2]  == ' ' and board[position[0]+1][position[1]-1] == ' 'and \
                    board[position[0]+1][position[1]-2] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0],position[1]-1],[position[0],position[1]-2]])  
                    match_found = True
                # plays feasible option 5
                elif board[position[0]][position[1]-1] == ' ' and board[position[0]-1][position[1]-1] == ' ' and \
                    board[position[0]-2][position[1]-1]== ' 'and board[position[0]+1][position[1]-1] == ' ' and \
                    board[position[0]+1][position[1]] == ' ' and board[position[0]][position[1]-2] == ' ' and \
                    board[position[0]+1][position[1]-2] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0],position[1]-1],[position[0]-1,position[1]-1]])   
                    match_found = True
                # plays feasible option 7
                elif board[position[0]+1][position[1]]  == ' ' and board[position[0]+2][position[1]]  == ' 'and \
                    board[position[0]+3][position[1]]  == ' 'and board[position[0]+1][position[1]+1]  == ' 'and \
                    board[position[0]+2][position[1]+1]  == ' ' and board[position[0]-1][position[1]+1] == ' 'and \
                    board[position[0]-2][position[1]+1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]-1,position[1]],[position[0]-2,position[1]]])  
                    match_found = True              
    
        elif top_half == board_domino[1]:
            if orientation == 'v':
            # plays feasible option 1
                if board[position[0]+1][position[1]+1] == ' ' and board[position[0]+1][position[1]+2] == ' ' and \
                board[position[0]][position[1]+1] == ' ' and board[position[0]][position[1]+2] == ' ' and \
                board[position[0]+2][position[1]+1]== ' ' and board[position[0]+2][position[1]+2] == ' 'and \
                board[position[0]+1][position[1]+3] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0]+1,position[1]+1],[position[0]+1,position[1]+2]])
                    match_found = True
                # plays feasible option 3
                elif board[position[0]+2][position[1]]  == ' ' and board[position[0]+2][position[1]+1]  == ' 'and \
                board[position[0]+2][position[1]+2]  == ' 'and board[position[0]+2][position[1]-1]  == ' 'and \
                board[position[0]+1][position[1]+1]  == ' ' and board[position[0]+3][position[1]] == ' 'and \
                board[position[0]+3][position[1]+1] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0]+2,position[1]],[position[0]+2,position[1]+1]]) 
                    match_found = True
                # plays feasible option 4
                elif board[position[0]+2][position[1]]  == ' ' and board[position[0]+3][position[1]]  == ' 'and \
                    board[position[0]+4][position[1]]  == ' 'and board[position[0]+2][position[1]+1]  == ' 'and \
                    board[position[0]+3][position[1]+1]  == ' ' and board[position[0]+2][position[1]-1] == ' 'and \
                    board[position[0]+3][position[1]-1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]+2,position[1]],[position[0]+3,position[1]]])  
                    match_found = True
                # plays feasible option 5
                elif board[position[0]+2][position[1]] == ' ' and board[position[0]+2][position[1]-1] == ' ' and \
                    board[position[0]+2][position[1]+1]== ' 'and board[position[0]+2][position[1]-2] == ' ' and \
                    board[position[0]+1][position[1]-1] == ' ' and board[position[0]+3][position[1]] == ' ' and \
                    board[position[0]+3][position[1]-1] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0]+2,position[1]],[position[0]+2,position[1]-1]])  
                    match_found = True 
                # plays feasible option 7
                elif board[position[0]+1][position[1]-1]  == ' ' and board[position[0]+1][position[1]-2]  == ' 'and \
                    board[position[0]+1][position[1]-3]  == ' 'and board[position[0]][position[1]-1]  == ' 'and \
                    board[position[0]][position[1]-2]  == ' ' and board[position[0]+2][position[1]-1] == ' 'and \
                    board[position[0]+2][position[1]-2] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0]+1,position[1]-1],[position[0]+1,position[1]-2]]) 
                    match_found = True
            elif orientation == 'h':
                # plays feasible option 2
                if board[position[0]-1][position[1]+1] == ' ' and board[position[0]-2][position[1]+1] == ' ' and \
                    board[position[0]-3][position[1]+1]== ' 'and board[position[0]-1][position[1]] == ' ' and \
                    board[position[0]+2][position[1]] == ' ' and board[position[0]+2][position[1]+2] == ' ' and \
                    board[position[0]+2][position[1]+3] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]-1,position[1]+1],[position[0]-2,position[1]+1]])   
                    match_found = True
                # plays feasible option 3
                elif board[position[0]-1][position[1]+2]  == ' ' and board[position[0]][position[1]+2]  == ' 'and \
                board[position[0]-2][position[1]+2]  == ' 'and board[position[0]+1][position[1]+2]  == ' 'and \
                board[position[0]-1][position[1]+3]  == ' ' and board[position[0]][position[1]+3] == ' 'and \
                board[position[0]-1][position[1]+1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]-1,position[1]+2],[position[0],position[1]+2]]) 
                    match_found = True
                # plays feasible option 4
                elif board[position[0]][position[1]+2]  == ' ' and board[position[0]][position[1]+3]  == ' 'and \
                    board[position[0]][position[1]+4]  == ' 'and board[position[0]-1][position[1]+2]  == ' 'and \
                    board[position[0]-1][position[1]+3]  == ' ' and board[position[0]+1][position[1]+2] == ' 'and \
                    board[position[0]+1][position[1]+3] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0],position[1]+2],[position[0],position[1]+3]])  
                    match_found = True
                # plays feasible option 5
                elif board[position[0]][position[1]+2] == ' ' and board[position[0]+1][position[1]+2] == ' ' and \
                    board[position[0]+2][position[1]+2]== ' 'and board[position[0]-1][position[1]+2] == ' ' and \
                    board[position[0]][position[1]+3] == ' ' and board[position[0]+1][position[1]+3] == ' ' and \
                    board[position[0]+1][position[1]+1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0],position[1]+2],[position[0]+1,position[1]+2]])   
                    match_found = True
                # plays feasible option 7
                elif board[position[0]+1][position[1]+1]  == ' ' and board[position[0]+2][position[1]+1]  == ' 'and \
                    board[position[0]+1][position[1]+2]  == ' 'and board[position[0]+2][position[1]+2]  == ' 'and \
                    board[position[0]+1][position[1]]  == ' ' and board[position[0]][position[1]+2] == ' 'and \
                    board[position[0]+3][position[1]+1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]+1,position[1]+1],[position[0]+2,position[1]+1]]) 
                    match_found = True
        elif bottom_half == board_domino[0]:
            if orientation == 'v':
                # plays feasible option 1
                if board[position[0]][position[1]+1]  == ' ' and board[position[0]][position[1]+2]  == ' 'and \
                board[position[0]][position[1]+3]  == ' 'and board[position[0]+1][position[1]+1]  == ' 'and \
                board[position[0]+1][position[1]+2]  == ' ' and board[position[0]-1][position[1]+1] == ' 'and \
                board[position[0]-1][position[1]+2] == '':
                    played_orientation = "h"
                    played_position = np.array([[position[0],position[1]+2],[position[0],position[1]+1]])
                    match_found = True
                # plays feasible option 3
                elif board[position[0]-1][position[1]]  == ' ' and board[position[0]-1][position[1]+1]  == ' 'and \
                board[position[0]-1][position[1]+2]  == ' 'and board[position[0]+1][position[1]+1]  == ' 'and \
                board[position[0]-2][position[1]]  == ' ' and board[position[0]-2][position[1]+1] == ' 'and \
                board[position[0]-1][position[1]-1] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0]-1,position[1]+1],[position[0]-1,position[1]]]) 
                    match_found = True
                # plays feasible option 4
                elif board[position[0]-1][position[1]-1]  == ' ' and board[position[0]-1][position[1]-2]  == ' 'and \
                    board[position[0]-1][position[1]-3]  == ' 'and board[position[0]-1][position[1]+1]  == ' 'and \
                    board[position[0]-2][position[1]+1]  == ' ' and board[position[0]-1][position[1]-1] == ' 'and \
                    board[position[0]-2][position[1]-1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]-2,position[1]],[position[0]-1,position[1]]])
                    match_found = True
                # plays feasible option 5
                elif board[position[0]-1][position[1]]  == ' ' and board[position[0]-1][position[1]-1]  == ' 'and \
                    board[position[0]-1][position[1]+1]  == ' 'and board[position[0]-1][position[1]-2]  == ' 'and \
                    board[position[0]][position[1]-1]  == ' ' and board[position[0]-2][position[1]] == ' 'and \
                    board[position[0]-2][position[1]-1] == '':
                    played_orientation = "h"
                    played_position = np.array([[position[0]-1,position[1]-1],[position[0]-1,position[1]]])
                    match_found = True
                # plays feasible option 7
                elif board[position[0]][position[1]-1]  == ' ' and board[position[0]][position[1]-2]  == ' 'and \
                    board[position[0]][position[1]-3]  == ' 'and board[position[0]-1][position[1]-1]  == ' 'and \
                    board[position[0]-1][position[1]-2]  == ' ' and board[position[0]+1][position[1]-1] == ' 'and \
                    board[position[0]+1][position[1]-2] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0],position[1]-2],[position[0],position[1]-1]])
                    match_found = True
            elif orientation == 'h':
                # plays feasible option 1
                if board[position[0]-1][position[1]] == ' ' and board[position[0]-2][position[1]] == ' ' and \
                board[position[0]-3][position[1]] == ' ' and board[position[0]-1][position[1]-1] == ' ' and \
                board[position[0]-2][position[1]-1]== ' ' and board[position[0]-1][position[1]+1] == ' 'and \
                board[position[0]-2][position[1]+1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]-2,position[1]],[position[0]-1,position[1]]])
                    match_found = True
                # plays feasible option 3
                elif board[position[0]][position[1]-1]  == ' ' and board[position[0]-1][position[1]-1]  == ' 'and \
                board[position[0]-1][position[1]-2]  == ' 'and board[position[0]+1][position[1]-1]  == ' 'and \
                board[position[0]-1][position[1]]  == ' ' and board[position[0]][position[1]-2] == ' 'and \
                board[position[0]-1][position[1]-2] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]-1,position[1]-1],[position[0],position[1]-1]]) 
                    match_found = True
                # plays feasible option 4
                elif board[position[0]][position[1]-1]  == ' ' and board[position[0]][position[1]-2]  == ' 'and \
                    board[position[0]][position[1]-3]  == ' 'and board[position[0]-1][position[1]-1]  == ' 'and \
                    board[position[0]-1][position[1]-2]  == ' ' and board[position[0]+1][position[1]-1] == ' 'and \
                    board[position[0]+1][position[1]-2] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0],position[1]-2],[position[0],position[1]-1]])  
                    match_found = True
                # plays feasible option 5
                elif board[position[0]][position[1]-1] == ' ' and board[position[0]-1][position[1]-1] == ' ' and \
                    board[position[0]-2][position[1]-1]== ' 'and board[position[0]+1][position[1]-1] == ' ' and \
                    board[position[0]+1][position[1]] == ' ' and board[position[0]][position[1]-2] == ' ' and \
                    board[position[0]+1][position[1]-2] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]-1,position[1]-1],[position[0],position[1]-1]]) 
                    match_found = True  
                # plays feasible option 7
                elif board[position[0]+1][position[1]]  == ' ' and board[position[0]+2][position[1]]  == ' 'and \
                    board[position[0]+3][position[1]]  == ' 'and board[position[0]+1][position[1]+1]  == ' 'and \
                    board[position[0]+2][position[1]+1]  == ' ' and board[position[0]-1][position[1]+1] == ' 'and \
                    board[position[0]-2][position[1]+1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]-2,position[1]],[position[0]-1,position[1]]])
                    match_found = True

        elif bottom_half == board_domino[1]:
            if orientation == 'v':
            # plays feasible option 1
                if board[position[0]+1][position[1]+1] == ' ' and board[position[0]+1][position[1]+2] == ' ' and \
                board[position[0]][position[1]+1] == ' ' and board[position[0]][position[1]+2] == ' ' and \
                board[position[0]+2][position[1]+1]== ' ' and board[position[0]+2][position[1]+2] == ' 'and \
                board[position[0]+1][position[1]+3] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0]+1,position[1]+2],[position[0]+1,position[1]+1]])
                    match_found = True
                # plays feasible option 3
                elif board[position[0]+2][position[1]]  == ' ' and board[position[0]+2][position[1]+1]  == ' 'and \
                board[position[0]+2][position[1]+2]  == ' 'and board[position[0]+2][position[1]-1]  == ' 'and \
                board[position[0]+1][position[1]+1]  == ' ' and board[position[0]+3][position[1]] == ' 'and \
                board[position[0]+3][position[1]+1] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0]+2,position[1]+1],[position[0]+2,position[1]]]) 
                    match_found = True
                # plays feasible option 4
                elif board[position[0]+2][position[1]]  == ' ' and board[position[0]+3][position[1]]  == ' 'and \
                    board[position[0]+4][position[1]]  == ' 'and board[position[0]+2][position[1]+1]  == ' 'and \
                    board[position[0]+3][position[1]+1]  == ' ' and board[position[0]+2][position[1]-1] == ' 'and \
                    board[position[0]+3][position[1]-1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]+3,position[1]],[position[0]+2,position[1]]])  
                    match_found = True
                # plays feasible option 5
                elif board[position[0]+2][position[1]] == ' ' and board[position[0]+2][position[1]-1] == ' ' and \
                    board[position[0]+2][position[1]+1]== ' 'and board[position[0]+2][position[1]-2] == ' ' and \
                    board[position[0]+1][position[1]-1] == ' ' and board[position[0]+3][position[1]] == ' ' and \
                    board[position[0]+3][position[1]-1] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0]+2,position[1]-1],[position[0]+2,position[1]]])   
                    match_found = True
                # plays feasible option 7
                elif board[position[0]+1][position[1]-1]  == ' ' and board[position[0]+1][position[1]-2]  == ' 'and \
                    board[position[0]+1][position[1]-3]  == ' 'and board[position[0]][position[1]-1]  == ' 'and \
                    board[position[0]][position[1]-2]  == ' ' and board[position[0]+2][position[1]-1] == ' 'and \
                    board[position[0]+2][position[1]-2] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0]+1,position[1]-2],[position[0]+1,position[1]-1]]) 
                    match_found = True
            elif orientation == 'h':
                # plays feasible option 2
                if board[position[0]-1][position[1]+1] == ' ' and board[position[0]-2][position[1]+1] == ' ' and \
                    board[position[0]-3][position[1]+1]== ' 'and board[position[0]-1][position[1]] == ' ' and \
                    board[position[0]+2][position[1]] == ' ' and board[position[0]+2][position[1]+2] == ' ' and \
                    board[position[0]+2][position[1]+3] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]-2,position[1]+1],[position[0]-1,position[1]+1]]) 
                    match_found = True  
                # plays feasible option 3
                elif board[position[0]][position[1]+2]  == ' ' and board[position[0]-1][position[1]+2]  == ' 'and \
                board[position[0]-2][position[1]+2]  == ' 'and board[position[0]+1][position[1]+2]  == ' 'and \
                board[position[0]-1][position[1]+3]  == ' ' and board[position[0]][position[1]+3] == ' 'and \
                board[position[0]-1][position[1]+1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0],position[1]+2],[position[0]-1,position[1]+2]]) 
                    match_found = True
                # plays feasible option 4
                elif board[position[0]][position[1]+2]  == ' ' and board[position[0]][position[1]+3]  == ' 'and \
                    board[position[0]][position[1]+4]  == ' 'and board[position[0]-1][position[1]+2]  == ' 'and \
                    board[position[0]-1][position[1]+3]  == ' ' and board[position[0]+1][position[1]+2] == ' 'and \
                    board[position[0]+1][position[1]+3] == ' ':
                    played_orientation = "h"
                    played_position = np.array([[position[0],position[1]+3],[position[0],position[1]+2]])  
                    match_found = True
                # plays feasible option 5
                elif board[position[0]][position[1]+2] == ' ' and board[position[0]+1][position[1]+2] == ' ' and \
                    board[position[0]+2][position[1]+2]== ' 'and board[position[0]-1][position[1]+2] == ' ' and \
                    board[position[0]][position[1]+3] == ' ' and board[position[0]+1][position[1]+3] == ' ' and \
                    board[position[0]+1][position[1]+1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]+1,position[1]+2],[position[0],position[1]+2]])   
                    match_found = True
                # plays feasible option 7
                elif board[position[0]+1][position[1]+1]  == ' ' and board[position[0]+2][position[1]+1]  == ' 'and \
                    board[position[0]+1][position[1]+2]  == ' 'and board[position[0]+2][position[1]+2]  == ' 'and \
                    board[position[0]+1][position[1]]  == ' ' and board[position[0]][position[1]+2] == ' 'and \
                    board[position[0]+3][position[1]+1] == ' ':
                    played_orientation = "v"
                    played_position = np.array([[position[0]+2,position[1]+1],[position[0]+1,position[1]+1]])
                    match_found = True

        if played_position is not None and np.all(played_position >= 0) and np.all(played_position < [10, 10]):
            return match_found, played_orientation, played_position
        else:
            return False, None, None


    ## Create 2 grids of the same size that will be filled with the center of mass position values of each grid
    # Creates 2 grids, one to store x values, and one to store y value. Postions calculated from top left corner of image
    ## Assumptions:
    # 1. picture is 720 by 720 pixels
    # 2. Each grid on the board will be 50 pixels wide
    # 3. We assume that the playing board will be 8 by 8 grids 
    # 4. This means, our playing board is 400 by 400 pixels
    # 5. Top left corner of playing board will be 150 pixels right of the top left corner of the image
    # 6. Top left corner of playing board will be 150 pixels below the top left corner of the image
    def grid_positions(self):
        grid_size = 50
        board_corner = np.array([150,150])
        x_cm = np.array([[board_corner[0]+(grid_size/2),board_corner[0]+(1.5*grid_size),
                        board_corner[0]+(2.5*grid_size),board_corner[0]+(3.5*grid_size),
                        board_corner[0]+(4.5*grid_size),board_corner[0]+(5.5*grid_size),
                        board_corner[0]+(6.5*grid_size),board_corner[0]+(7.5*grid_size)]]*12)
        y_cm = x_cm.T
        return x_cm, y_cm

    def hand_converter(self, msg):
        self.num_hand_dominos = msg.num_dominos
        self.hand_dots_half1 = msg.num_dots_half1
        self.hand_dots_half2 = msg.num_dots_half2
        self.hand_dom_x_cm = msg.x
        self.hand_dom_y_cm = msg.y
        self.hand_dom_orientation = msg.orientation

        self.hand_dots_half1 = np.array(self.hand_dots_half1)
        self.hand_dots_half2 = np.array(self.hand_dots_half2)
        
        self.hand_dom_cm = np.vstack((np.array(self.hand_dom_x_cm),np.array(self.hand_dom_y_cm)))

    def board_converter(self, msg):
        self.num_board_dominos = msg.num_dominos
        self.board_dots_half1 = msg.num_dots_half1
        self.board_dots_half2 = msg.num_dots_half2
        self.board_dom_x_cm = msg.x
        self.board_dom_y_cm = msg.y
        self.board_dom_orientation = msg.orientation

        board_dots_half1 = np.array(self.board_dots_half1)
        board_dots_half2 = np.array(self.board_dots_half2)
        
        self.board_dom_cm = np.vstack((np.array(self.board_dom_x_cm),np.array(self.board_dom_y_cm)))
        

    def game_engine(self):
        rospy.Subscriber("/board_info",game_state, self.board_converter)
        rospy.Subscriber("/hand_info",game_state, self.hand_converter)
        rospy.spin()

              
        
    
        board = self.initialize_board()
        x_cm, y_cm = self.grid_positions()
        print(self.board(board))


        turn_over = False

        ## Initialize client node where we would extract the game state value
        # Filler values for hand dominoes and their positions
        hand_dom = np.vstack((self.hand_dots_half1,self.hand_dots_half2))
        hand_pos_cm = self.hand_dom_
        while not turn_over: 
            # Filler values for board dominoes and their positions
            board_dom = np.vstack((self.board_dots_half1,self.board_dots_half2))
            # Initializes positions of board dominoes on computer's grid
            board_pos = np.array([[4,3,2,0],
                                [3,2,1,1],
                                [4,5,2,1],
                                [4,2,2,1]])
            board_dom_orientation = self.board_dom_orientation

            for i in range(np.size(board_dom,1)):
                self.place_domino(board, board_dom[:,i], board_pos[0,i], board_pos[1,i], board_dom_orientation[i])
            
            print(self.board(board))

            hand_size = np.size(hand_dom,1)
            board_size = np.size(board_dom,1)

            adjacent_domino = np.zeros(2)
            played_domino = np.zeros(2)
            played_domino_position = np.zeros(2)

            i = 0
            j = 0
            valid = False #indication that no match has been found

            while i < hand_size:
                potential_domino = hand_dom[:,i]
                top_half = potential_domino[0]
                bottom_half = potential_domino[1]
                j = 0
                while j < board_size:
                    #if turn_state == 0:
                    # Checks current domino in hand to every domino on the board and see if there is a match
                    if top_half == board_dom[0, j] or top_half == board_dom[1, j] or \
                            bottom_half == board_dom[0, j] or bottom_half == board_dom[1, j]:
                        
                        #Checks which side of the board domino matches and will determine feasibility
                        adjacent_domino = np.array([board_dom[0,j],board_dom[1,j]]) #domino we are going to place our domino next to
                        adjacent_orientation = board_dom_orientation[j]
                        adjacent_pos = np.array([board_pos[0,j],board_pos[1,j]]) #position of the top of the domino
                        
                        match_found,played_orientation, played_position=valid_move(board, potential_domino, adjacent_domino, adjacent_pos, adjacent_orientation)
                        
                        
                        if match_found == True:
                            domino_Hand_Position = hand_pos_cm[:,i] #Where the domino is located in the robot's hand
                            #placed_domino_position = 
                            valid = True #Indicates that a match has been found
                            print("Board Domino is ",  adjacent_domino)
                            print("Played Domino is ", potential_domino)
                            print("We will place the domino at", played_position)
                            print("The domino will have an orientation of", played_orientation)

                        #Include code to pick up domino from the domino position, and play it in a feasible location

                        turn_over = True
                        break
                    else:
                        j += 1
                if valid:
                    break
                i +=1
            if not valid:
                #We need to add code relating to picking up a domino from the boneyard
                print("No match found, picking domino from the boneyard")

                #Placeholder for picking up a random domino
                new_random_dominoes = np.random.randint(1, 7, size=(2, 1))
                new_random_domino_pos = np.random.randint(150,900, size =(2,1))

                # Concatenate the random array with the existing hand_dom array
                hand_dom = np.concatenate((hand_dom, new_random_dominoes), axis=1)
                hand_pos_cm = np.concatenate((hand_pos_cm, new_random_domino_pos), axis=1)
                print(hand_dom)
        else:
            print('It is now the player turn')



if __name__ == "__main__":
    a = GameEngine()

        