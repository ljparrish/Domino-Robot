
#!/usr/bin/env python

import numpy as np
import random
import rospy
from domino_vision_pkg.msg import game_state, hand_pos #msg type
# import ar_track_alvar
import tf2_ros
import tf
from geometry_msgs.msg import PoseStamped, Point, Quarternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class GameEngine:
    def __init__(self):
        
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

        self.gridbrainpos_xhalf1 = None
        self.gridbrainpos_xhalf2 = None
        self.gridbrainpos_yhalf1 = None
        self.gridbrainpos_yhalf2 = None
        

        rospy.Subscriber("/board_info",game_state, self.board_converter)
        rospy.Subscriber("/hand_info",game_state, self.hand_converter)
        rospy.Subscriber("/robot/joint_states",JointState, self.robo_joint_states)


        rospy.spin()
    ## Create an empty grid that will be filled in with domino values
    def initialize_board(self):
        # Initialize a 2D array for the domino board
        return [[' ' for _ in range(8)] for _ in range(8)]

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

        if played_position is not None and np.all(played_position >= 0) and np.all(played_position < [8, 8]):
            return match_found, played_orientation, played_position
        else:
            return False, None, None


    ## Create 2 grids of the same size that will be filled with the center of mass position values of each grid
    # Creates 2 grids, one to store x values, and one to store y value. Postions calculated from top left grid's center 
    
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

        self.board_dots_half1 = np.array(self.board_dots_half1)
        self.board_dots_half2 = np.array(self.board_dots_half2)
        
        self.board_dom_cm = np.vstack((np.array(self.board_dom_x_cm),np.array(self.board_dom_y_cm)))
    def robo_joint_states(self,msg):
        angles = msg.position      
        self.wrist_angle = angles[7]  

    def grid_positions(self):
        cell_size = 0.031
        board_corner = np.array([0.825,0.149])
        self.world_x_cm = np.array([[board_corner[0],board_corner[0]+cell_size,
                        board_corner[0]+(2*cell_size),board_corner[0]+(3*cell_size),
                        board_corner[0]+(4*cell_size),board_corner[0]+(5*cell_size),
                        board_corner[0]+(6*cell_size),board_corner[0]+(7*cell_size)]])
        self.world_y_cm = np.array([[board_corner[1],board_corner[1]+cell_size,
                        board_corner[1]+(2*cell_size),board_corner[1]+(3*cell_size),
                        board_corner[1]+(4*cell_size),board_corner[1]+(5*cell_size),
                        board_corner[1]+(6*cell_size),board_corner[1]+(7*cell_size)]])

    def grid_brain(self):
        # Get positions of cm of domino halves in real-world
        # compare their coordinate value (x,y distance) with the known grid positions
        # 2 for loops: 1st one goes through each domino, 2nd one compares to each of the grid positions
        for index1, a in enumerate(self.board_dom_x_cm):
            for index2, b in enumerate(self.world_x_cm):
                if abs(b-a) < 0.031/2.1: # Made it 2.1 instead of 2 to make the threshold smaller
                    if index1 % 2 == 0:
                        self.gridbrainpos_xhalf1.append(index2)
                    else: 
                        self.gridbrainpos_xhalf2.append(index2)
        
        for index3, a in enumerate(self.board_dom_y_cm):
            for index4, b in enumerate(self.world_y_cm):
                if abs(b-a) < 0.031/2.1: # Made it 2.1 instead of 2 to make the threshold smaller
                    if index3 % 2 == 0:
                        self.gridbrainpos_yhalf1.append(index4)
                    else: 
                        self.gridbrainpos_yhalf2.append(index4)

        self.gridbrainpos_xhalf1 = np.array(self.gridbrainpos_xhalf1) 
        self.gridbrainpos_xhalf2 = np.array(self.gridbrainpos_xhalf2) 
        self.gridbrainpos_yhalf1 = np.array(self.gridbrainpos_yhalf1) 
        self.gridbrainpos_yhalf2 = np.array(self.gridbrainpos_yhalf2)     
        self.gridbrainpos = np.vstack((self.gridbrainpos_xhalf1,self.gridbrainpos_xhalf2,self.gridbrainpos_yhalf1,self.gridbrainpos_yhalf2))
    
    def grid_to_world(self, played_position): ## SEAN WRITE STUFF HERE        
        # Take as input the grid indices x,y positions for each half and return the domino's center of mass for both halves, then calculate full center of mass (average)
        # played position is a 2x2 with row 1 half 1: x,y. Row 2/half2: x,y
        # if horizontal, calculate center of mass this way. # if vertical, calculate center of mass other way. 
        des_board_dom_cm = []

        h1x = self.world_x_cm[played_position[0,0]]
        h2x = self.world_x_cm[played_position[1,0]]
        h1y = self.world_y_cm[played_position[0,1]]
        h2y = self.world_y_cm[played_position[1,1]]
        des_board_dom_cm[0] = np.mean(h1x,h2x) # x position 
        des_board_dom_cm[1] = np.mean(h1y,h2y) # y position 

        return des_board_dom_cm
    
    def game_engine(self):

        board = self.initialize_board()
        print(self.board(board))

        turn_over = False

        hand_dom = np.vstack((self.hand_dots_half1,self.hand_dots_half2))
        hand_pos_cm = self.hand_dom_cm
        while not turn_over: 
            # Filler values for board dominoes and their positions
            board_dom = np.vstack((self.board_dots_half1,self.board_dots_half2))
            # Initializes positions of board dominoes on computer's grid
            '''
            board_pos = np.array([[4,3,2,0],
                                [3,2,1,1],
                                [4,5,2,1],
                                [4,2,2,1]]) old example that Will made
            ''' 
            board_pos = self.gridbrainpos

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
                        adjacent_orientation = board_dom_orientation[j] #orientation of the domino we are going to place our domino next to
                        adjacent_pos = np.array([board_pos[0,j],board_pos[1,j]]) #position of the top of the domino
                        
                        match_found,played_orientation, played_position=self.valid_move(board, potential_domino, adjacent_domino, adjacent_pos, adjacent_orientation)
                        
                        
                        if match_found == True:
                            
                        ## Publishes the position of the domino in the hand that we want to get
                            domino_height = -0.120 #How far domino is from gripper before getting picked up

                            if top_half == board_dom[0,j] or top_half == board_dom[1,j]:
                                desired_dom_hand_pos = np.array([hand_pos_cm[:,i],hand_pos_cm[:,i+1]]) #Where the domino is located in the robot's hand
                            else:
                                desired_dom_hand_pos = np.array([hand_pos_cm[:,i-1],hand_pos_cm[:,i]]) #Where the domino is located in the robot's hand    
                            
                            #Takes the center of mass of both halves of the domino and calculates the center of mass of the actual domino
                            hand_dom_cm = np.array([np.mean(desired_dom_hand_pos[0,0], desired_dom_hand_pos[0,1]),np.mean(desired_dom_hand_pos[0,0], desired_dom_hand_pos[1,0])])
                            
                            pick_up_pose = PoseStamped()
                            pick_up_pose.header = Header(stamp=rospy.Time.now(), frame_id="base")
                            pick_up_pose.pose.position.x = hand_dom_cm[0]
                            pick_up_pose.pose.position.y = hand_dom_cm[1]
                            pick_up_pose.pose.position.z = domino_height
                            pick_up_pose.pose.orientation.x = 0.0
                            pick_up_pose.pose.orientation.y = 1.0
                            pick_up_pose.pose.orientation.z = 0.0
                            pick_up_pose.pose.orientation.w = 0.0


                            '''self.hand_pub= rospy.Publisher('/desired_hand_pos',hand_pose, queue_size = 10)
                            r = rospy.Rate(10)      
                            hand_pub_string = hand_pose
                            self.hand_pub.publish(hand_pub_string)
                            r.sleep()'''
                        

                        ## Publishes the Pose of the where we want to place the domino on the board
                            # Set Quarternion for the end effector pose
                            if played_orientation == 'h':
                                if top_half == board_dom[0,j] or top_half == board_dom[1,j]:
                                    #Do not rotate end effector
                                    self.wrist_angle = self.wrist_angle
                                elif bottom_half == board_dom[0,j] or top_half==board_dom[1,j]:
                                    #Rotate end effector by 180 degrees
                                    self.wrist_angle = self.wrist_angle + np.deg2rad(180) 
                            elif played_orientation == 'v':
                                if top_half == board_dom[0,j] or top_half == board_dom[1,j]:
                                    #Rotate end effector by 90 degrees
                                    self.wrist_angle = self.wrist_angle + np.deg2rad(90)
                                elif bottom_half == board_dom[0,j] or top_half==board_dom[1,j]:
                                    #Rotate end effector by 270 degrees
                                    self.wrist_angle = self.wrist_angle + np.deg2rad(270)
                            quat_x,quat_y,quat_z,quat_w = tf.transformations.quaternion_from_euler(0,0,self.wrist_angle, 'ryxz')
                            
                            ## Calculate the center of mass of where we want to place the domino
                            des_board_dom_cm = self.grid_to_world(played_position)
                            #placed_domino_position = 
                            place_pose = PoseStamped()
                            place_pose.header = Header(stamp=rospy.Time.now(), frame_id="base")
                            place_pose.pose.position.x = des_board_dom_cm[0]
                            place_pose.pose.position.y = des_board_dom_cm[1]
                            place_pose.pose.position.z = domino_height
                            place_pose.pose.orientation.x = quat_x
                            place_pose.pose.orientation.y = quat_y
                            place_pose.pose.orientation.z = quat_z
                            place_pose.pose.orientation.w = quat_w

                            '''
                            self.def_board_pub= rospy.Publisher('/desired_board_pos',des_board_pose, queue_size = 10)
                            r = rospy.Rate(10)      
                            des_board_pub_string = des_board_pose
                            self.def_board_pub.publish(des_board_pub_string)
                            r.sleep()
                            '''


                            valid = True #Indicates that a match has been found

                            
                            print("Board Domino is ",  adjacent_domino)
                            print("Played Domino is ", potential_domino)
                            print("We will place the domino at", place_pose)
                            print("The domino will have an orientation of", played_orientation)
                            # combine played_position and played_orientation
                            turn_over = True
                            return match_found, pick_up_pose, place_pose

                        break
                    else:
                        j += 1
                if valid:
                    break
                i +=1
            return match_found, None, None




if __name__ == "__main__":
    rospy.init_node('game_engine', anonymous = True)
    GameEngine()

        