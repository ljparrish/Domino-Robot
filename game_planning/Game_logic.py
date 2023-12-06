import numpy as np
import random
import scipy.linalg



## Create an empty grid that will be filled in with domino values
def initialize_board():
    # Initialize a 2D array for the domino board
    return [[' ' for _ in range(8)] for _ in range(8)]

def print_board(board):
    # Print the current state of the domino board
    for row in board:
        print('|' + '|'.join(str(cell) if cell != ' ' else ' ' for cell in row) + '|')
        print('-' * 17)

# Function places domino at specified position in the array
def place_domino(board, domino, row1, row2, col1, col2, orientation):
    # Place the domino at the specified position on the board
    if orientation == 'h':
        board[row1][col1] = domino[0]
        board[row2][col2] = domino[1]
    elif orientation == 'v':
        board[row1][col1] = domino[0]
        board[row2][col2] = domino[1]



def valid_move(board, hand_domino, board_domino, position, orientation):
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
    feasible_option = 0

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
               board[position[0]-2][position[1]-1] == ' ' and board[position[0]-1][position[1]+1] == ' ' and \
               board[position[0]-2][position[1]+1] == ' ':
                played_orientation = "v"
                played_position = np.array([[position[0]-1,position[1]],[position[0]-2,position[1]]])
                match_found = True
                feasible_option = 1
            # plays feasible option 3
            elif board[position[0]][position[1]-1]  == ' ' and board[position[0]-1][position[1]-1]  == ' 'and \
               board[position[0]-1][position[1]-2]  == ' 'and board[position[0]+1][position[1]-1]  == ' 'and \
               board[position[0]-1][position[1]]  == ' ' and board[position[0]][position[1]-2] == ' 'and \
               board[position[0]-1][position[1]-2] == ' ':
                played_orientation = "v"
                played_position = np.array([[position[0],position[1]-1],[position[0]-1,position[1]-1]]) 
                match_found = True
                feasible_option = 3
            # plays feasible option 4
            elif board[position[0]][position[1]-1]  == ' ' and board[position[0]][position[1]-2]  == ' 'and \
                 board[position[0]][position[1]-3]  == ' 'and board[position[0]-1][position[1]-1]  == ' 'and \
                 board[position[0]-1][position[1]-2]  == ' ' and board[position[0]+1][position[1]-1] == ' 'and \
                 board[position[0]+1][position[1]-2] == ' ':
                played_orientation = "h"
                played_position = np.array([[position[0],position[1]-1],[position[0],position[1]-2]])  
                match_found = True
                feasible_option = 4
            # plays feasible option 5
            elif board[position[0]][position[1]-1] == ' ' and board[position[0]-1][position[1]-1] == ' ' and \
                 board[position[0]-2][position[1]-1]== ' 'and board[position[0]+1][position[1]-1] == ' ' and \
                 board[position[0]+1][position[1]] == ' ' and board[position[0]][position[1]-2] == ' ' and \
                 board[position[0]+1][position[1]-2] == ' ':
                played_orientation = "v"
                played_position = np.array([[position[0],position[1]-1],[position[0]-1,position[1]-1]])   
                match_found = True
                feasible_option = 5
            # plays feasible option 7
            elif board[position[0]+1][position[1]]  == ' ' and board[position[0]+2][position[1]]  == ' 'and \
                 board[position[0]+3][position[1]]  == ' 'and board[position[0]+1][position[1]+1]  == ' 'and \
                 board[position[0]+2][position[1]+1]  == ' ' and board[position[0]-1][position[1]+1] == ' 'and \
                 board[position[0]-2][position[1]+1] == ' ':
                played_orientation = "v"
                played_position = np.array([[position[0]+1,position[1]],[position[0]+2,position[1]]])  
                match_found = True    
                feasible_option = 7          
   
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
                played_position = np.array([[position[0]-1,position[1]-2],[position[0]-1,position[1]-1]])
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
                played_position = np.array([[position[0]+2,position[1]],[position[0]+1,position[1]]])
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
                played_position = np.array([[position[0]-1,position[1]+2],[position[0],position[1]+2]]) 
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
        return match_found, played_orientation, played_position, feasible_option
    else:
        return False, None, None, feasible_option


## Create 2 grids of the same size that will be filled with the center of mass position values of each grid
# Creates 2 grids, one to store x values, and one to store y value. Postions calculated from top left corner of image
## Assumptions:

def grid_positions():
    cell_size = 0.031
    cell_size2 = 0.028
    board_corner = np.array([0.825,0.149])
    grid_x_cm = np.array([board_corner[0],board_corner[0]-cell_size,
                    board_corner[0]-(2*cell_size),board_corner[0]-(3*cell_size),
                    board_corner[0]-(4*cell_size),board_corner[0]-(5*cell_size),
                    board_corner[0]-(6*cell_size),board_corner[0]-(7*cell_size)])
    grid_y_cm = np.array([board_corner[1],board_corner[1]-cell_size2,
                    board_corner[1]-(2*cell_size2),board_corner[1]-(3*cell_size2),
                    board_corner[1]-(4*cell_size2),board_corner[1]-(5*cell_size2),
                    board_corner[1]-(6*cell_size2),board_corner[1]-(7*cell_size2)])
    
    return grid_x_cm, grid_y_cm

def grid_brain(grid_x_cm, grid_y_cm, board_x_cm, board_y_cm):
    # Get positions of cm of domino halves in real-world
    # compare their coordinate value (x,y distance) with the known grid positions
    # 2 for loops: 1st one goes through each domino, 2nd one compares to each of the grid positions
    brainpos_xhalf1 = []
    brainpos_xhalf2 = []
    brainpos_yhalf1 = []
    brainpos_yhalf2 = []
    '''threshold1 = 0.014
    threshold2 = 0.014
    for index1 in range(np.size(board_x_cm)):
        for index2 in range(np.size(grid_x_cm)):
            if abs(board_x_cm[index1]-grid_x_cm[index2]) <= threshold1:
                if index1 % 2 == 0:
                    brainpos_xhalf1.append(index2)
                    #print(gridbrainpos_xhalf1)
                else: 
                    brainpos_xhalf2.append(index2)
    for index3 in range(np.size(board_y_cm)):
        for index4 in range(np.size(grid_y_cm)):
            if (abs(grid_y_cm[index4]-board_y_cm[index3]) <= threshold2): # Made it 2.1 instead of 2 to make the threshold smaller
            #if (0.04*index4 < 0.05):
                if index3 % 2 == 0:
                    brainpos_yhalf1.append(index4)
                else: 
                    brainpos_yhalf2.append(index4)'''

    for numDominos2 in range(len(board_x_cm)):
        distanceError = 100
        best_X_index = 0
        best_Y_index = 0
        for idx in range(len(grid_x_cm)):
            for ydx in range(len(grid_y_cm)):
                newError = np.sqrt(scipy.linalg.norm([board_x_cm[numDominos2] - grid_x_cm[idx], board_y_cm[numDominos2] - grid_y_cm[ydx]]))
                if newError < distanceError:
                    distanceError = newError
                    best_X_index = idx
                    best_Y_index = ydx
                    print("new error = ",newError)
        if numDominos2 % 2 == 0:
            brainpos_xhalf1.append(best_X_index)
        else: 
            brainpos_xhalf2.append(best_X_index)
        if numDominos2 % 2 == 0:
            brainpos_yhalf1.append(best_Y_index)
        else: 
            brainpos_yhalf2.append(best_Y_index)
        

                




    brainpos_xhalf1 = np.array(brainpos_xhalf1) 
    brainpos_xhalf2 = np.array(brainpos_xhalf2) 
    brainpos_yhalf1 = np.array(brainpos_yhalf1) 
    brainpos_yhalf2 = np.array(brainpos_yhalf2)
    print(brainpos_xhalf1)
    print(brainpos_yhalf1)
    print(brainpos_xhalf2)
    print(brainpos_yhalf2)
    gridbrainpos = np.vstack((brainpos_xhalf1,brainpos_yhalf1,brainpos_xhalf2,brainpos_yhalf2))
    return gridbrainpos
def grid_to_world(played_position, grid_x_cm, grid_y_cm):      
    # Take as input the grid indices x,y positions for each half and return the domino's center of mass for both halves, then calculate full center of mass (average)
    # played position is a 2x2 with row 1 half 1: x,y. Row 2/half2: x,y
    # if horizontal, calculate center of mass this way. # if vertical, calculate center of mass other way. 
    des_board_dom_cm = np.zeros(2)

    h1x = grid_x_cm[played_position[0,0]]
    h2x = grid_x_cm[played_position[1,0]]
    h1y = grid_y_cm[played_position[0,1]]
    h2y = grid_y_cm[played_position[1,1]]
    des_board_dom_cm[0] = (h1x+h2x)/2 # x position 
    des_board_dom_cm[1] = (h1y+h2y)/2 # y position 

    return des_board_dom_cm
def main():
    board = initialize_board()
    
    grid_x_cm, grid_y_cm = grid_positions()
    print(grid_y_cm)
    board_x_offset = 0
    board_y_offset = 0
    """board_x_cm = np.array([0.709666, 0.709835, 0.75435, 0.735])+board_x_offset
    board_y_cm = np.array([0.06142, 0.04205, 0.0638, 0.0636])+board_y_offset"""
    #print(board_y_cm)
    #gridbrainpos = grid_brain(grid_x_cm, grid_y_cm, board_x_cm, board_y_cm)
    


    turn_over = False

    ## Initialize client node where we would extract the game state value
    # Filler values for hand dominoes and their positions
    hand_dom = np.array([[1,5,1,5,1,2],
                        [2,3,1,2,4,0]])
    hand_pos_cm = np.array([[850,855,851,852,853],
                            [100,150,200,250,300]])
    while not turn_over: 
        # Filler values for board dominoes and their positions
        board_dom = np.array([[4,5,4,4,4],
                              [3,4,0,0,0]])
        # Initializes positions of board dominoes on computer's grid
        board_pos = np.array([[2,4,4,1,2],
                              [3,3,2,3,1],
                              [3,4,4,0,2],
                              [3,4,1,3,2]])
        board_orientations = ["h", "v", "h","v","h"]

        for i in range(np.size(board_dom,1)):
            place_domino(board, board_dom[:,i], board_pos[0,i], board_pos[2,i], board_pos[1,i], board_pos[3,i], board_orientations[i])
        
        print_board(board)
        print(board[0][0])

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
                    adjacent_orientation = board_orientations[j]
                    adjacent_pos = np.array([board_pos[0,j],board_pos[1,j]]) #position of the matching side of the domino
                    
                    # Played position outputs a 2x2 array. The first row tells is the position of one half of the domino.
                    match_found,played_orientation, played_position, feasible_option = valid_move(board, potential_domino, adjacent_domino, adjacent_pos, adjacent_orientation)
                      
                    
                    if match_found == True:
                        domino_Hand_Position = hand_pos_cm[:,i] #Where the domino is located in the robot's hand
                        #placed_domino_position = 
                        valid = True #Indicates that a match has been found
                        place_domino(board, hand_dom[:,i], played_position[0,0], played_position[1,0], played_position[0,1],played_position[1,1], played_orientation)
                        print(print_board(board))
                        des_board_dom_cm = grid_to_world(played_position, grid_x_cm, grid_y_cm)
                        print("Board Domino is ",  adjacent_domino)
                        print("Played Domino is ", potential_domino)
                        print(feasible_option)
                        print("We will place the domino at", played_position)
                        print("Real world placement will be at ", des_board_dom_cm)
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
    main()