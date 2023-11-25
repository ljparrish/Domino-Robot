import numpy as np
import random

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
def place_domino(board, domino, row, col, orientation):
    # Place the domino at the specified position on the board
    if orientation == 'h':
        board[row][col] = domino[0]
        board[row][col + 1] = domino[1]
    elif orientation == 'v':
        board[row][col] = domino[0]
        board[row + 1][col] = domino[1]

def valid_move(board, domino, position):
    """
    Check if placing a domino on the board at the specified position is feasible.

    Parameters:
    - board: The current state of the domino board.
    - domino: The domino to be placed.
    - position: The position on the board where the top-left corner of the domino will be placed.

    Returns:
    - feasibility: True if the placement is feasible, False otherwise.
    - orientation: The preferred orientation ('h' for horizontal, 'v' for vertical).
    """
    rows, cols = np.shape(board)
    top_half, bottom_half = domino

    # Check feasibility for both orientations
    h_feasible = v_feasible = True

    # Check horizontal feasibility
    if position[1] + 1 < cols and board[position[0]][position[1] + 1] == ' ':
        h_feasible = True
    else:
        h_feasible = False

    # Check vertical feasibility
    if position[0] + 1 < rows and board[position[0] + 1][position[1]] == ' ':
        v_feasible = True
    else:
        v_feasible = False

    # Determine preferred orientation based on feasibility
    if h_feasible and v_feasible:
        # If both orientations are feasible, choose one randomly
        orientation = random.choice(['h', 'v'])
    elif h_feasible:
        orientation = 'h'
    elif v_feasible:
        orientation = 'v'
    else:
        # Both orientations are not feasible
        return False, None

    return True, orientation


## Create 2 grids of the same size that will be filled with the center of mass position values of each grid
# Creates 2 grids, one to store x values, and one to store y value. Postions calculated from top left corner of image
## Assumptions:
# 1. picture is 720 by 720 pixels
# 2. Each grid on the board will be 50 pixels wide
# 3. We assume that the playing board will be 8 by 8 grids 
# 4. This means, our playing board is 400 by 400 pixels
# 5. Top left corner of playing board will be 150 pixels right of the top left corner of the image
# 6. Top left corner of playing board will be 150 pixels below the top left corner of the image
def grid_positions():
    grid_size = 50
    board_corner = np.array([150,150])
    x_cm = np.array([[board_corner[0]+(grid_size/2),board_corner[0]+(1.5*grid_size),
                      board_corner[0]+(2.5*grid_size),board_corner[0]+(3.5*grid_size),
                      board_corner[0]+(4.5*grid_size),board_corner[0]+(5.5*grid_size),
                      board_corner[0]+(6.5*grid_size),board_corner[0]+(7.5*grid_size)]]*8)
    y_cm = x_cm.T
    return x_cm, y_cm


def main():
    board = initialize_board()
    x_cm, y_cm = grid_positions()
    print_board(board)


    turn_over = False

    ## Initialize client node where we would extract the game state value
    # Filler values for hand dominoes and their positions
    hand_dom = np.array([[1,1,4,4,1,4,1],
                        [4,4,1,1,1,4,4]])
    hand_pos_cm = np.array([[850,855,851,852,853,855,850],
                            [100,150,200,250,300,350,400]])
    while not turn_over: 
        # Filler values for board dominoes and their positions
        board_dom = np.array([[3,6,2,0],
                              [5,3,6,2]])
        # Initializes positions of board dominoes on computer's grid
        board_pos = np.array([[4,3,2,0],
                              [3,2,1,1],
                              [4,5,2,1],
                              [4,2,2,1]])
        board_orientations = ["h", "v", "h", "v"]

        for i in range(np.size(board_dom,1)):
            place_domino(board, board_dom[:,i], board_pos[0,i], board_pos[1,i], board_orientations[i])
        
        print_board(board)

        hand_size = np.size(hand_dom,1)
        board_size = np.size(board_dom,1)

        adjacent_domino = np.zeros(2)
        played_domino = np.zeros(2)
        played_domino_position = np.zeros(2)

        i = 0
        j = 0
        match_found = False #indication that no match has been found

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
                    if top_half == board_dom[0, j]:
                        if board_orientations[j] == 'v':
                            ## Input feasibility code to see if we can place domino there
                            feasibility, preferred_orientation = valid_move(board, potential_domino, (board_pos[0, j], board_pos[1, j]))
                        else:
                            ## Input feasibility code to see if we can place domino there
                            feasibility, preferred_orientation = valid_move(board, potential_domino, (board_pos[0, j], board_pos[1, j]))
                    elif top_half == board_dom[1, j]:
                        if board_orientations[j] == 'v':
                            ## Input feasibility code to see if we can place domino there
                            feasibility, preferred_orientation = valid_move(board, potential_domino, (board_pos[0, j], board_pos[1, j]))
                        else:
                            ## Input feasibility code to see if we can place domino there
                            feasibility, preferred_orientation = valid_move(board, potential_domino, (board_pos[0, j], board_pos[1, j]))
                    elif bottom_half == board_dom[0, j]:
                        if board_orientations[j] == 'v':
                            ## Input feasibility code to see if we can place domino there
                            feasibility, preferred_orientation = valid_move(board, potential_domino, (board_pos[0, j], board_pos[1, j]))
                        else:
                            ## Input feasibility code to see if we can place domino there
                            feasibility, preferred_orientation = valid_move(board, potential_domino, (board_pos[0, j], board_pos[1, j]))
                    elif bottom_half == board_dom[1, j]:
                        if board_orientations[j] == 'v':
                            ## Input feasibility code to see if we can place domino there
                            feasibility, preferred_orientation = valid_move(board, potential_domino, (board_pos[0, j], board_pos[1, j]))
                        else:
                            ## Input feasibility code to see if we can place domino there
                            feasibility, preferred_orientation = valid_move(board, potential_domino, (board_pos[0, j], board_pos[1, j]))

                    
                    

                    domino_Hand_Position = hand_pos_cm[:,i] #Where the domino is located in the robot's hand
                    #placed_domino_position = 
                    match_found = True #Indicates that a match has been found
                    print("Board Domino is ",  adjacent_domino)
                    print("Played Domino is ", potential_domino)
                    print("The Domino we will pick up is at", domino_Hand_Position)

                    #Include code to pick up domino from the domino position, and play it in a feasible location

                    turn_over = True
                    break
                else:
                    j += 1
            if match_found:
                break
            i +=1
        if not match_found:
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